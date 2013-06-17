/*
 * cloud_player_widget.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: asher
 */

#include <QMenu>
#include <qfiledialog.h>
#include <qerrormessage.h>

#include <pcl/visualization/cloud_player_widget.h>
#include <pcl/io/movie_grabber.h>
#include <pcl/io/pcd_recorder.h>

pcl::visualization::CloudPlayerWidget::CloudPlayerWidget (QWidget* parent,
    Qt::WindowFlags f) :
    QWidget (parent, f), current_renderer_idx_ (2), playing_ (false),
    recording_ (false), is_movie_grabber_ (false), pcl_visualizer_ (NULL),
    error_msg_ (new QErrorMessage)
{

  ui_.setupUi (this);
  this->ui_.button_record->setPopupMode (QToolButton::MenuButtonPopup);
  this->ui_.button_record->setMenu (new QMenu);

  this->ui_.button_play_pause->setPopupMode (QToolButton::MenuButtonPopup);
  this->ui_.button_play_pause->setMenu (new QMenu);

  pcl_visualizer_ = new pcl::visualization::PCLVisualizer ("", false);
  ui_.qvtkwidget->SetRenderWindow (pcl_visualizer_->getRenderWindow ());
  pcl_visualizer_->setupInteractor (ui_.qvtkwidget->GetInteractor (),
      ui_.qvtkwidget->GetRenderWindow ());
  ui_.qvtkwidget->GetInteractor ()->SetInteractorStyle (
      pcl_visualizer_->getInteractorStyle ());
  ui_.qvtkwidget->update ();
  ui_.qvtkwidget->GetInteractor ()->RemoveObservers (vtkCommand::ExitEvent);

  pcl_visualizer_->registerKeyboardCallback (
      boost::bind (&CloudPlayerWidget::keyboardCB, this, _1));

  addCloudRenderer (new CloudRendererRange ("x"));
  addCloudRenderer (new CloudRendererRange ("y"));
  addCloudRenderer (new CloudRendererRange ("z"));

  QObject::connect (ui_.button_play_pause, SIGNAL(clicked()), this,
      SLOT(playPause()));
  QObject::connect (ui_.button_record, SIGNAL(clicked()), this, SLOT(record()));
  QObject::connect (ui_.button_home, SIGNAL(clicked()), this,
      SLOT(resetView()));

  this->setWindowTitle ("Cloud Player");

  this->addRecorder (new PCDRecorder ());

  resetView ();
}

pcl::visualization::CloudPlayerWidget::~CloudPlayerWidget ()
{
  for (int i = 0; i < recorders_.size (); i++)
    delete recorders_[i];
  for (int i = 0; i < renderers_.size (); i++)
    delete renderers_[i];
  delete error_msg_;
}

void pcl::visualization::CloudPlayerWidget::setGrabber (
    boost::shared_ptr<Grabber>& grabber)
{
  assert(grabber!=NULL);
  if (playing_)
    playPause ();  //pause the playing

  this->grabber_ = grabber;
  is_movie_grabber_ = (dynamic_cast<MovieGrabber*> (& (*grabber)) != NULL);

  cache_connection_.disconnect ();
  typedef void (sig_cb_cloud) (
      const boost::shared_ptr<const sensor_msgs::PointCloud2>&);
  cache_connection_ = grabber->registerCallback<sig_cb_cloud> (
      boost::bind (&CloudPlayerWidget::cacheCloud, this, _1));

  this->ui_.button_play_pause->setEnabled (true);
  this->ui_.button_record->setEnabled (true);

  if (is_movie_grabber_)
    enablePlayback ();
  else
    disablePlayback ();
}

void pcl::visualization::CloudPlayerWidget::enableRenderering ()
{
  if (grabber_ == NULL)
    return;
  if (!this->renderers_[current_renderer_idx_]->setup (grabber_))
  {
    std::string emsg = "Current Coloring \""
        + this->renderers_[current_renderer_idx_]->description ();
    emsg = emsg + "\" not valid for " + grabber_->getName ()
        + " using Z Range coloring instead\n";
    error_msg_->showMessage (emsg.c_str ());
    error_msg_->show ();
    current_renderer_idx_ = 2;
    this->renderers_[current_renderer_idx_]->setup (grabber_);
  }
}

void pcl::visualization::CloudPlayerWidget::addCloudRenderer (
    CloudRenderer* renderer)
{
  renderer->init (ui_.qvtkwidget, pcl_visualizer_);
  renderers_.push_back (renderer);

  QAction* render_action = new QAction (renderer->description ().c_str (),
      NULL);
  connect (render_action, SIGNAL(triggered()), this,
      SLOT(rendererSelectedViaMenu()));
  this->ui_.button_play_pause->menu ()->addAction (render_action);
}

void pcl::visualization::CloudPlayerWidget::cacheCloud (
    const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  cached_cloud_ = cloud;
}

void pcl::visualization::CloudPlayerWidget::keyboardCB (
    const pcl::visualization::KeyboardEvent& event)
{

}

void pcl::visualization::CloudPlayerWidget::rendererSelectedViaMenu ()
{
  for (int i = 0; i < ui_.button_play_pause->menu ()->actions ().size (); i++)
  {
    if (sender () == ui_.button_play_pause->menu ()->actions ()[i])
    {
      setCurrentRenderer (i);
      break;
    }
  }

  enableRenderering ();
  if (cached_cloud_ != NULL)
  {
    renderers_[current_renderer_idx_]->setCloud (cached_cloud_);
    renderers_[current_renderer_idx_]->renderNew ();
    this->update ();
  }
}

void pcl::visualization::CloudPlayerWidget::playPause ()
{
  assert(grabber_!=NULL);

  if (playing_)
  {  //now pause
    this->ui_.button_play_pause->setIcon (QIcon (":/viewer/imgs/play.png"));
    playing_ = false;
    grabber_->stop ();
  }
  else
  {
    setCurrentRenderer (current_renderer_idx_);
    enableRenderering ();
    this->ui_.button_play_pause->setIcon (QIcon (":/viewer/imgs/pause.png"));
    playing_ = true;
    grabber_->start ();
  }
}

void pcl::visualization::CloudPlayerWidget::record ()
{
  if (recording_)
  {
    recorders_[current_recorder_idx_]->stop ();
    this->ui_.button_record->setIcon (
        QIcon (":/viewer/imgs/recorddisabled.png"));
    this->ui_.button_record->menu ()->setEnabled (true);
    return;
  }

  if (sender () != ui_.button_record)
  {
    for (int i = 0; i < ui_.button_record->menu ()->actions ().size (); i++)
    {
      if (sender () == ui_.button_record->menu ()->actions ()[i])
      {
        current_recorder_idx_ = i;
        break;
      }
    }
  }

  recorders_[current_recorder_idx_]->setGrabber (grabber_);

  if (!recorders_[current_recorder_idx_]->hasValidGrabber ())
  {
    error_msg_->showMessage (
        "Error : Input stream is not supported by selected recorder.");
    return;
  }

  boost::filesystem::path record_path =
      QFileDialog::getSaveFileName (NULL,
          tr ("Choose the root file/folder of the recording."), "", "").toAscii ().data ();

  if (record_path.empty ())
    return;

  record_path.replace_extension ("");

  recorders_[current_recorder_idx_]->setOutput (
      record_path.parent_path ().string (), record_path.filename ().string (),
      0);
  recorders_[current_recorder_idx_]->start ();
  this->ui_.button_record->setIcon (QIcon (":/viewer/imgs/recordpressed.png"));
  this->ui_.button_record->menu ()->setEnabled (false);
  recording_ = true;
}

void pcl::visualization::CloudPlayerWidget::resetView ()
{
#if ( ( PCL_MAJOR_VERSION >=1) && (  PCL_MINOR_VERSION > 6) )
  pcl_visualizer_->setCameraPosition (0, 0, -2, 0, 0, 1, 0, -1, 0);
#else
  pcl_visualizer_->setCameraPose (0, 0, -2,
      0, 0, 1,
      0,-1,0);
#endif
  this->ui_.qvtkwidget->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->ResetCameraClippingRange ();
  this->ui_.qvtkwidget->GetRenderWindow ()->Render ();
}

void pcl::visualization::CloudPlayerWidget::enablePlayback ()
{
  MovieGrabber* mg = dynamic_cast<MovieGrabber*> (& (*grabber_));
  QObject::connect (ui_.progress_bar, SIGNAL(valueChanged(int)), this,
      SLOT(sliderValueChanged(int)));
  this->ui_.progress_bar->setEnabled (true);
  this->ui_.progress_bar->setMaximum (mg->getFrameCount () - 1);
  progress_connection_ = this->grabber_->registerCallback<
      MovieGrabber::sig_frame_num_cb> (
      boost::bind (&CloudPlayerWidget::progressUpdate, this, _1, _2));
  this->ui_.progress_bar->setValue (0);
}

void pcl::visualization::CloudPlayerWidget::disablePlayback ()
{
  this->ui_.progress_bar->setEnabled (false);
  progress_connection_.disconnect ();
  this->ui_.progress_bar->setValue (0);
}

void pcl::visualization::CloudPlayerWidget::sliderValueChanged (int val)
{
  MovieGrabber* mg = dynamic_cast<MovieGrabber*> (& (*grabber_));
  mg->setFrameNumber (val);
  mg->playOneFrame ();
}

void pcl::visualization::CloudPlayerWidget::updateCloud ()
{
  this->renderers_[current_renderer_idx_]->renderNew ();
}

pcl::visualization::CloudRenderer* pcl::visualization::CloudPlayerWidget::getRenderer (
    size_t idx)
{
  if (idx >= renderers_.size ())
    return NULL;
  return renderers_[idx];
}

void pcl::visualization::CloudPlayerWidget::setCurrentRenderer (int idx)
{
  if ( (idx >= renderers_.size ()) || (idx < 0))
    return;
  this->renderers_[current_renderer_idx_]->disconnect ();
  disconnect (this->renderers_[current_renderer_idx_], SIGNAL(update()), this,
      SLOT(updateCloud()));
  if (grabber_ != NULL)
    this->renderers_[current_renderer_idx_]->setup (grabber_);

  current_renderer_idx_ = idx;
  connect (this->renderers_[current_renderer_idx_], SIGNAL(update()), this,
      SLOT(updateCloud()));
}

void pcl::visualization::CloudPlayerWidget::addRecorder (Recorder* recorder)
{
  assert(recorder!=NULL);
  recorders_.push_back (recorder);

  QAction* record_action = new QAction (recorder->getDescription ().c_str (),
      NULL);
  connect (record_action, SIGNAL(triggered()), this, SLOT(record()));
  this->ui_.button_record->menu ()->addAction (record_action);
}

void pcl::visualization::CloudPlayerWidget::setCurrentRecorder (uint32_t idx)
{
  if (recorders_.size () <= idx)
    return;
  current_recorder_idx_ = idx;
}

void pcl::visualization::CloudPlayerWidget::progressUpdate (size_t frame_num,
    size_t frame_total)
{
  MovieGrabber* mg = dynamic_cast<MovieGrabber*> (& (*grabber_));
  this->ui_.progress_bar->setMinimum (0);
  this->ui_.progress_bar->setMaximum (frame_total);
  this->ui_.progress_bar->setValue (frame_num);
}

