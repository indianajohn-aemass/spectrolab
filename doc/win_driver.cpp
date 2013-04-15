#define _WIN32_WINNT 0x501 
#define ID_STARTLP 199
#define ID_STARTHP 200
#define ID_STOP  201
#define ID_SEND  202
#define WM_SOCKMISC 104
#define WM_SOCKIMG 105
#define WM_IMGSCKT 140
#define RX_PORT "4000\0"
#define RX_PORTIMG "4002\0"
#define TX_PORT "4950\0"
#define MAXBUFLEN 1024
#define HAVE_REMOTE
#define SER_DELAY 64000
#undef _WIN64

#include <WinSock2.h>
#include <fstream>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ws2tcpip.h>
#include <string>
#include <IPHlpApi.h>
//#include <pcap\pcap.h> 
//#include "pcap.h"

#pragma comment(lib, "Ws2_32.lib")
//#pragma comment(lib,"wpcap.lib")
#pragma comment(lib,"Iphlpapi.lib")
const char rx_ip_address[20] = "192.168.0.121\0";
const char tx_ip_address[20] = "192.168.0.27\0";       //// from image_receive project

//using namespace std;
int InitWinsockListener(HWND hwnd);
int InitWinsockListenerIMG(HWND hwnd);
int removeWhite(char text [],int length);
int parse(char *text,int length);
void delay(unsigned int i_loops);
int udp_sender(char instruction[] ,int size);
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
static char gszClassName[]  = "LadarInterface";
int udp_listener();
static HINSTANCE ghInstance = NULL;
void copy(char *src,char*dst,int length);
void dwordFlip(char * pacbuff);
char * convert(wchar_t * item);
HWND combo;
char mess[256] ;
int linecount=0;
int sockrcv;
int sockimg;
char filepath[100]="frame.dat\0";//"C:\\Users\\alee\\Desktop\\frame.dat\0";
char Filename[30]="Frame0000\0";
char FrameBuff[131073];
int FbufCount=0;
std::ofstream file;
FILE *fw;
long convBuff=0;
char sendBuffer[256]={0x27,0x00,0x00,-1};///null terminated(-1)
int delay_ms = 10;

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow){
        WNDCLASSEX WndClass;
        HWND hwnd;
        MSG Msg;
		//Filename[16]='\0';
        /////////////////////////////////
        AllocConsole() ;
        AttachConsole( GetCurrentProcessId() );/////////////////// attaches a console for easier debugging
        freopen( "CON","w",stdout);
				
        //////////////////////////////////////
                
        char a;
        WSADATA wsaData;

        //MAKEWORD(1,1) for Winsock 1.1, MAKEWORD(2,0) for Winsock 2.0:
        if (WSAStartup(MAKEWORD(2,0), &wsaData) != 0){
				fprintf(stderr, "WSAStartu6p failed.\n");   // winsock init
			exit(1);
        }

		fw=fopen(Filename, "wb");

        ghInstance = hInstance;
        WndClass.cbSize        = sizeof(WNDCLASSEX);
        WndClass.style         = NULL;
        WndClass.lpfnWndProc   = WndProc;
        WndClass.cbClsExtra    = 0;
        WndClass.cbWndExtra    = 0;
        WndClass.hInstance     = ghInstance;
        WndClass.hIcon         = LoadIcon(NULL, IDI_APPLICATION);
        WndClass.hCursor       = LoadCursor(NULL, IDC_ARROW);
        WndClass.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
        WndClass.lpszMenuName  = NULL;
        WndClass.lpszClassName =(LPCTSTR) gszClassName;
        WndClass.hIconSm       = LoadIcon(NULL, IDI_APPLICATION);

        if(!RegisterClassEx(&WndClass)) {
                MessageBox(0, L"Error Registering Window!", L"Error!", MB_ICONSTOP | MB_OK);
                return 0;
        }

        hwnd = CreateWindowEx(
                WS_EX_STATICEDGE,
                (LPCTSTR)gszClassName,
                L"Ladar Link",
                WS_OVERLAPPEDWINDOW,
                CW_USEDEFAULT, CW_USEDEFAULT,
                700, 375,
                NULL, NULL,
                ghInstance,
                NULL);

        if(hwnd == NULL) {
                MessageBox(0, L"Window Creation Failed!", L"Error!", MB_ICONSTOP | MB_OK);
                return 0;
        }

        ShowWindow(hwnd, nCmdShow);
        UpdateWindow(hwnd);

        while(GetMessage(&Msg, NULL, 0, 0)) {
                TranslateMessage(&Msg);
                DispatchMessage(&Msg);
        }
        return Msg.wParam;
}

LRESULT CALLBACK WndProc(HWND hwnd, UINT Message, WPARAM wParam, LPARAM lParam) {
        HWND hButtonStart,hButtonStop, hCombo,hButtonSend, hEdit, hList, hScroll, hStatic,hConsole;
        int Index;
		int	size=0;
		char instruct[5]= {0x27,0x00,0x00};
		hConsole=GetConsoleWindow();
		LONG lStyle;
		TCHAR Item[256]={-1};
		LPSTR tBuf[30];
		char padapters[5000];
		PIP_ADAPTER_INFO adapters;
		MIB_IPNETROW ArpRow;
		ULONG AdapIndex;
		ULONG bufflen;
		BYTE mac[8]={0x00,0x0F,0xCC,0x23,0x00,0x01};
		int i;
		int size1 = 2;//
				
        switch(Message){
				case WM_SOCKMISC:{
							if(WSAGETSELECTERROR(lParam))
								{	
									MessageBox(hwnd,
										(LPCWSTR)"Connection to server failed",
										L"Error",
										MB_OK|MB_ICONERROR);
									SendMessage(hwnd,WM_DESTROY,NULL,NULL);
									break;
								}
							switch(WSAGETSELECTEVENT(lParam))
							{
								case FD_READ:
								{
									char szIncoming[1024];
									ZeroMemory(szIncoming,sizeof(szIncoming));

									int inDataLength=recv(sockrcv,
										(char*)szIncoming,
										sizeof(szIncoming)/sizeof(szIncoming[0]),
										0);
									std::cout<<"received ";
									for(int i=0;i<12;i++){
										std::cout<<std::hex<<(unsigned __int16)szIncoming[i];
										if(i%2==1)
											std::cout<<" ";
									}
									std::cout<<" from adc"<<std::endl;
									std::cout<<"\t received ";
									for(int i=0;i< inDataLength-12;i++){
										std::cout<<std::hex<<(unsigned __int16)szIncoming[i+12]<<" ";
									}
									std::cout<<"from SPI"<<std::endl;
								}
								break;
							}	
						}break;
					case WM_SOCKIMG:{
							if(WSAGETSELECTERROR(lParam))
								{	
									MessageBox(hwnd,
										(LPCWSTR)"Connection to server failed",
										L"Error",                                        //// have to work on
										MB_OK|MB_ICONERROR);
									SendMessage(hwnd,WM_DESTROY,NULL,NULL);
									break;
								}
							switch(WSAGETSELECTEVENT(lParam))
							{
								case FD_READ:
								{
									char szIncoming[1024];
									ZeroMemory(szIncoming,sizeof(szIncoming));

									int inDataLength=recv(sockimg,
										(char*)szIncoming,
										sizeof(szIncoming)/sizeof(szIncoming[0]),///////////////////////////////////////////
										0);
									if(inDataLength!=1024){
										MessageBox(hwnd,
										(LPCWSTR)"Frame Packet not 1024 bytes long",
										L"Error",                                        //// image save line by line at file "Frames/Frame0000" where the
										MB_OK|MB_ICONERROR);}							//// 0's are replaced by frame number
									if((szIncoming[0]==(char)152)&&(szIncoming[1]==(char)186)&&(szIncoming[2]==(char)220)&&(szIncoming[3]==(char)254)){//98 BA DC FE
										if(!(linecount<127))
										{	fclose(fw);
											filepath[0]='\0';
											fwrite(filepath,1,1,fw);
											fclose(fw);
											/////////////////////////////////////////////////////////////////file.close();
											FbufCount=0;
											convBuff++;
										

											Filename[5]=(char)((convBuff)/1000+48);
											Filename[6]=(char)(((convBuff)%1000)/100+48);
											Filename[7]=(char)((((convBuff)%1000)%100)/10+48);
											Filename[8]=(char)((((convBuff)%1000)%100)%10+48);
											//std::ofstream file;
											//delete(shortFlip);
											std::cout<<"received frame"<<std::endl;
											fw=fopen(Filename,"wb");//file.open(Filename,std::ios::out);//file.open(Filename,std::ios::binary);
											linecount=0;
										dwordFlip(szIncoming);
										//for(int i =0;i<1025;i++){
											fwrite(szIncoming,1024,1,fw);//file <<(char)szIncoming[i];
											//FrameBuff[FbufCount]=(char)szIncoming[i];
											//FbufCount++;
										//}
										}
										else{
											fclose(fw);
											filepath[0]='\0';
											fwrite(filepath,1,1,fw);
											fclose(fw);//file.close();
											FbufCount=0;
											Filename[5]=(char)((convBuff)/1000+48);
											Filename[6]=(char)(((convBuff)%1000)/100+48);
											Filename[7]=(char)((((convBuff)%1000)%100)/10+48);
											Filename[8]=(char)((((convBuff)%1000)%100)%10+48);
											//std::ofstream file;
											//delete(shortFlip);
											fw=fopen(Filename,"wb");//file.open(Filename,std::ios::out);//file.open(Filename,std::ios::binary);
											linecount=0;
										dwordFlip(szIncoming);
										//for(int i =0;i<1025;i++){
											fwrite(szIncoming,1024,1,fw);//file <<(char)szIncoming[i];
											//FrameBuff[FbufCount]=(char)szIncoming[i];
											//FbufCount++;
										//}
										}
										
									}
									else{if(linecount<127){
									dwordFlip(szIncoming);
									//for(int i =0;i<1025;i++){
										fwrite(szIncoming,1024,1,fw);//file <<(char)szIncoming[i];
										//FrameBuff[FbufCount]=(char)szIncoming[i];
										//FbufCount++;}
									linecount++;}
									else{
										//std::cout<<"over size"<<std::endl;
									}}
									
									
								}
								break;
							}	
						}break;


                case WM_CREATE:
								
								freopen("CON","r",stdin);
								size=0;
								bufflen=sizeof(padapters);
								 GetAdaptersInfo((PIP_ADAPTER_INFO)padapters,&bufflen);
								for(adapters=(PIP_ADAPTER_INFO)padapters;adapters;adapters=adapters->Next){
									std::cout<<size<<" ) "<<adapters->Description<<std::endl;
									size++;
								}
								std::cout<<"choose the number of the adapter that you would like to use"<<std::endl;
								std::cin>>size;
								adapters=(PIP_ADAPTER_INFO)padapters;
								size=0;
								for(i =0;i<size;i++)
									adapters=adapters->Next;
									//GetAdapterIndex((LPWSTR)adapters->AdapterName,&AdapIndex);
									ArpRow.dwIndex=adapters->Index;
									ArpRow.dwType=(DWORD)MIB_IPNET_TYPE_STATIC;
									ArpRow.dwAddr=(DWORD)inet_addr(tx_ip_address);
									ArpRow.dwPhysAddrLen=(DWORD)6;
									ArpRow.bPhysAddr[0]=(UCHAR)mac[0];
									ArpRow.bPhysAddr[1]=(UCHAR)mac[1];
									ArpRow.bPhysAddr[2]=(UCHAR)mac[2];
									ArpRow.bPhysAddr[3]=(UCHAR)mac[3];
									ArpRow.bPhysAddr[4]=(UCHAR)mac[4];
									ArpRow.bPhysAddr[5]=(UCHAR)mac[5];
									i=(int)CreateIpNetEntry(&ArpRow);
									if(!(i==0|i==5010)){
										std::cout<<i<<std::endl;
								}




								hConsole=GetConsoleWindow();
								lStyle = GetWindowLong(hConsole, GWL_STYLE);
								lStyle &= ~(WS_CAPTION | WS_THICKFRAME | WS_MINIMIZE | WS_MAXIMIZE | WS_SYSMENU);
								SetWindowLong(hConsole, GWL_STYLE, lStyle);

								
								SetParent(hConsole,hwnd);
								SetWindowPos(hConsole,HWND_TOP,0,0,690,295,SWP_SHOWWINDOW);
								UpdateWindow(hConsole);
								
								if(InitWinsockListener(hwnd)==-1){
									std::cout<<"Winsock Listener Failed"<<std::endl;
									exit(1);//// research exit codes
								}

								if(InitWinsockListenerIMG(hwnd)==-1){
									std::cout<<"Winsock Listener Failed"<<std::endl;
									exit(1);//// research exit codes
								}

                        hButtonStart = CreateWindowEx(
                                NULL, L"Button", L"Start HP", WS_BORDER | WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                                455, 305, 80, 30,
                                hwnd, (HMENU)ID_STARTHP,
                                ghInstance,
                                NULL);
						hButtonSend = CreateWindowEx(
								NULL,L"Button",L"Start LP",WS_BORDER |WS_CHILD | WS_VISIBLE| BS_PUSHBUTTON,
								535, 305, 80, 30,
								hwnd,(HMENU)ID_STARTLP,
								ghInstance,
								NULL);
						hButtonStop = CreateWindowEx(
								NULL,L"Button",L"Stop", WS_BORDER |WS_CHILD | WS_VISIBLE| BS_PUSHBUTTON,
								605, 305, 60, 30,
								hwnd,(HMENU)ID_STOP,
								ghInstance,
								NULL);                                             
						hButtonSend = CreateWindowEx(
								NULL,L"Button",L"Send",WS_BORDER |WS_CHILD | WS_VISIBLE| BS_PUSHBUTTON,
								300, 305, 155, 30,
								hwnd,(HMENU)ID_SEND,
								ghInstance,
								NULL);
                        hCombo  = CreateWindowEx(
                                NULL,
                                L"ComboBox",
                                L"Commands",
                                WS_BORDER | WS_CHILD | WS_VISIBLE |CBS_DROPDOWN|CBS_HASSTRINGS |WS_OVERLAPPED,
                                0, 305, 300, 250,
                                hwnd,NULL,
                                ghInstance,
                                NULL);
						combo=hCombo;
                                                SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x27 00 00");
                                                SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x21");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x1A");
                                                SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x15");
                                                SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x17");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x12");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x11");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x18");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x16");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x13");
												SendMessage(hCombo,CB_ADDSTRING,0,(LPARAM)L"0x10");
												SendMessage(hCombo,CB_SETCURSEL,0,0);
                
                        break;
                case WM_COMMAND:
                        switch(HIWORD(wParam)){
                                case CBN_SELENDOK: 
                                        Index = SendMessage((HWND) lParam,(UINT) CB_GETCURSEL,(WPARAM)0,(LPARAM)0);
                                        memset( Item, 0,256 );
                                        (TCHAR) SendMessage((HWND)lParam,(UINT)CB_GETLBTEXT,(WPARAM)Index,(LPARAM)Item);
										size=0;
										for(int i=0;i<256;i++){
											if(Item[i]==(TCHAR)NULL)
												break;
											size++;
										}
										convert(Item);
										size=removeWhite(mess,size);
														
										size=parse(mess,size);
														 
										copy(mess,sendBuffer,size);
										sendBuffer[size]=NULL; 
                                        break;

								case CBN_EDITCHANGE:
										memset( Item, 0,256 );
										GetWindowText((HWND)lParam,(LPWSTR) Item,(UINT)30);
										size=0;
										for(int i=0;i<256;i++){
											if(Item[i]==(TCHAR)NULL)
												break;
											size++;
										}
										convert(Item);
										size=removeWhite(mess,size);
														  
										size=parse(mess,size);
										copy(mess,sendBuffer,size);
										// sendBuffer[size]=NULL;     //// parse and store text                                                       
										break;
                                case BN_CLICKED:{
										switch(LOWORD(wParam)){
										case (HMENU) ID_STARTHP :	
											//int size1 = 2;//
											mess[0]=0x3A;//
											mess[1]=-1;
											udp_sender(mess,size1);//
											delay(SER_DELAY);//
											mess[0]=0x3F;//
											mess[1]=-1;
											udp_sender(mess,size1);//
											delay(SER_DELAY);
											mess[0]=0x21;
											mess[1]=-1;
											//mess[1]=0x00;//NULL;
											//mess[2]=0x00;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x1A;
											mess[1]=-1;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x15;
											mess[1]=-1;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x17;
											mess[1]=-1;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x12;
											mess[1]=-1;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x11;
											mess[1]=-1;
											udp_sender(mess,size1);
											break;
										case (HMENU) ID_STARTLP :	
											size1 = 2;//
											mess[0]=0x3A;//
											mess[1]=-1;
											udp_sender(mess,size1);//
											delay(SER_DELAY);//
											mess[0]=0x3F;//
											udp_sender(mess,size1);//
											delay(SER_DELAY);
											mess[0]=0x21;
											//mess[1]=0x00;//NULL;
											//mess[2]=0x00;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x15;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x17;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x12;
											udp_sender(mess,size1);
											delay(SER_DELAY);
											mess[0]=0x11;
											udp_sender(mess,size1);
											break;
										case (HMENU) ID_SEND :	//size=GetWindowTextLength(combo)+1;
											//GetWindowText(combo,(LPWSTR) Item,(UINT)size);
											if(sendBuffer[0]==-1){
												MessageBox(hwnd, (LPCWSTR) sendBuffer, TEXT(" is Incorrect Syntax"), MB_OK);
												break;
											}//// need to do parsing
											//MessageBox(hwnd, (LPCWSTR) sendBuffer, TEXT(" is correct Syntax"), MB_OK);
											size=0;
											for(int i=0;i<256;i++){
												if(sendBuffer[i]==-1)
														break;
														size++;
												}		
																				
											udp_sender(sendBuffer,size);//udp_staticSend(instruct,size);
											break;
										case (HMENU) ID_STOP :	
											size1 = 2;
											mess[0]=0x18;
											mess[1]=-1;
											udp_sender(mess,size1);//udp_staticSend(instruct,size);
											delay(SER_DELAY);
											mess[0]=0x16;
											mess[1]=-1;
											udp_sender(mess,size1);//udp_staticSend(instruct,size);
											delay(SER_DELAY);
											mess[0]=0x13;
											mess[1]=-1;
											udp_sender(mess,size1);//udp_staticSend(instruct,size);
											delay(SER_DELAY);
											mess[0]=0x10;
											mess[1]=-1;
											udp_sender(mess,size1);//udp_senderStop();
											break;
										default:		std::cout<<"nothing"<<std::endl<<wParam<<std::endl<<lParam;
										}
										break;}
																
						}
                        break;
												
                case WM_CLOSE:
                        DestroyWindow(hwnd);
                        break;
                case WM_DESTROY:
						file.close();
                        PostQuitMessage(0);
                        break;
                default:
                         return DefWindowProc(hwnd, Message, wParam, lParam);
        }

        return 0;
}

int udp_sender(char instruction [],int size){

	std::cout<<"start sequence  ";
	for(int i=0;i<size;i++){
		std::cout<<std::hex<<(int)instruction[i]<<" ";
	}

	std::cout<<"sent\t";
	if(size<5)
		std::cout<<"\t";
	if(size<3)
		std::cout<<"\t";

    int sockfd;
	const int si=size;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
	char message[10];//MAXBUFLEN];

	for (int i=0;i<size;i++){
		message[i]=instruction[i];
	}

	memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    if ((rv = getaddrinfo(tx_ip_address, TX_PORT, &hints, &servinfo)) != 0){
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and make a socket
    for(p = servinfo; p != NULL; p = p->ai_next){
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1){
            perror("talker: socket");
            continue;
        }
        break;
    }

    if (p == NULL){
        fprintf(stderr, "talker: failed to bind socket\n");
        return 2;
    }

    if ((numbytes = sendto(sockfd, message, size, 0,
             p->ai_addr, p->ai_addrlen)) == -1){///////////////actually send
        perror("talker: sendto");
        exit(1);
    }

    freeaddrinfo(servinfo);

    printf("talker: sent %d bytes to %s\n", numbytes, tx_ip_address);
    //close(sockfd);
	closesocket(sockfd);
    return 0;
}


int removeWhite(char text [],int length){
	
	//char temp[256]={0};
	int j=0;

	for(int i =0 ;i<length;i++){
		if(!(text[i]==' '||text[i]=='	'||text[i]==13)){//(text[i]>0x2F&&text[i]<0x3A){
			mess[j]=text[i];
			j++;
		}
	}
	mess[j]=NULL;
	return j;
}

int parse(char *text,int length){
	///// use something that takes to chars subtracts 30 from each then shifts first one over 4
	///// this then ORs with the second one
	///// then trim the size so that array is not big
	if(text[0]!='0')
		return NULL;
	if(text[1]!='x'&&text[1]!='X')
		return NULL;;
	//char temp[256];
	int j =0;
	char high,low;

	for (int i =2;i<length-1;i++){
		if(text[i]>'/'&&text[i]<':'){
			high=(text[i]-0x30);}
		else if(text[i]>'@'&&text[i]<'G'){
			high=(text[i]-64)+9;}
		else if(text[i]>'`'&&text[i]<'g'){
			high=(text[i]-96)+9;}
		if(text[i+1]>'/'&&text[i+1]<':'){
			low=(text[i+1]-0x30);}
		else if(text[i+1]>'@'&&text[i+1]<'G'){
			low=(text[i+1]-64)+9;}
		else if(text[i+1]>'`'&&text[i+1]<'g'){
			low=(text[i+1]-96)+9;}
		mess[j]=(char)(high<<4)|(low);
		i++;
		j++;
	}
	mess[j]=-1;
	return j+1;
}





int InitWinsockListener(HWND hwnd){
	
	int nResult;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    struct sockaddr_storage their_addr;
    char buf[MAXBUFLEN + 1];
	int addr_len;
	int WordCount = 0;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP
	hints.ai_protocol = IPPROTO_UDP;

    if ((rv = getaddrinfo("192.168.0.121\0", RX_PORT, &hints, &servinfo)) != 0){
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next){
        if ((sockrcv = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1){
            perror("listener: socket");
            continue;
        }

        if (bind(sockrcv, p->ai_addr, p->ai_addrlen) == -1){
            //close(sockfd);
			closesocket(sockrcv);
            perror("listener: bind");
            continue;
        }
        break;
    }

    if (p == NULL){
        fprintf(stderr, "listener: failed to bind socket\n");
        return 2;
    }

    freeaddrinfo(servinfo);

    printf("listener: waiting to recvfrom...\n");
	//char buftemp[1024];
	//recv(sockrcv,buftemp,17,0);

	nResult=WSAAsyncSelect(sockrcv,hwnd,WM_SOCKMISC,FD_READ);
	if(nResult){
		MessageBox(hwnd,
			(LPCWSTR)"WSAAsyncSelect failed",
			L"Critical Error",
			MB_ICONERROR);
		SendMessage(hwnd,WM_DESTROY,NULL,NULL);
		return -1;;
	}
			
	return 0;
}


int InitWinsockListenerIMG(HWND hwnd){
	
	int nResult;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    struct sockaddr_storage their_addr;
    char buf[MAXBUFLEN + 1];
	int addr_len;
	int WordCount = 0;


    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP
	hints.ai_protocol = IPPROTO_UDP;


    if ((rv = getaddrinfo("192.168.0.121\0", RX_PORTIMG, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next){
        if ((sockimg = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("listener: socket");
            continue;
        }
        if (bind(sockimg, p->ai_addr, p->ai_addrlen) == -1){
            //close(sockfd);
			closesocket(sockimg);
            perror("listener: bind");
            continue;
        }
        break;
    }

    if (p == NULL) {
        fprintf(stderr, "listener: failed to bind socket\n");
        return 2;
    }

    freeaddrinfo(servinfo);

    printf("listener: waiting to recvfrom...\n");
	//char buftemp[1024];
	//recv(sockrcv,buftemp,17,0);

	nResult=WSAAsyncSelect(sockimg,hwnd,WM_SOCKIMG,FD_READ);
	if(nResult){
		MessageBox(hwnd,
			(LPCWSTR)"WSAAsyncSelect failed",
			L"Critical Error",
			MB_ICONERROR);
		SendMessage(hwnd,WM_DESTROY,NULL,NULL);
		return -1;;
	}
	return 0;
}

void copy(char *src,char*dst,int length){
	 
	for(int i=0;i<length;i++){
		 dst[i]=src[i];
	 }	
	 return;
}

char * convert(wchar_t * item){
	
	int i =0;
	
	while(item[i]!=NULL){
		mess[i]=(char)item[i];
		i++;
	}
	return NULL;
}

void dwordFlip(char * pacbuff){
	
	char temp;
	
	for(int i=0;i<1024;i=i+4){
		temp=pacbuff[i];
		pacbuff[i]=pacbuff[i+2];
		pacbuff[i+2]=temp;
		temp=pacbuff[i+1];
		pacbuff[i+1]=pacbuff[i+3];
		pacbuff[i+3]=temp;
	}
}

void delay(unsigned int i_loops){
	unsigned int i;

	for(i = 0; i <= i_loops; i++);

}
