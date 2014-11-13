//Copyright 2014 University of Kentucky
//Nkiruka Uzuegbunam, 11/10/2014

#define _CRT_SECURE_NO_WARNINGS //to allow me to use sprintf, and fopen, and printf without warnings

//Windows headers
#include <Windows.h>

//General headers
//#include <fcntl.h> 
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
// Headers for OpenNI2 --OLD OpenNI2
#include <OpenNI.h>
// Headers for Kinect using OpenNI2: UnComment for x32
//#include <NiTE.h>
//The above require property sheet OpenNIProperty.props
// WIN32

#endif

#ifdef _WIN64
// Headers for OpenNI2 --new OpenNI2 from structure.io
#include <OpenNI.h>

//The above requires property sheet OpenNI_2014_x64_PropertySheets.props
// Headers for OpenCV2410
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>  
#endif // x64



//Global Variables for display
int window_w = 640;
int window_h = 480;


//Global variables for saving
std::string fileName;
std::string imagePrefix("img");
std::string depthPrefix("dep");
std::string imgEnd(".jpg");
std::string depthEnd(".dat");
int imgIndex = 0;//track the images saved
int imgCount = 0;//track all the images made
int timeStep = 10;//save every 10th image

//Global variables for openni
openni::Device device;//kinect device
openni::VideoStream depth, color; //videostreams
openni::VideoStream** m_streams;//for efficiency later
openni::VideoFrameRef depthFrame, colorFrame;//frames to read from polling agent
CONST INT NUM_STREAMS = 2;//number of streams for kinect


//functions 
bool HandleStatus(openni::Status status); //error handling
char ReadLastCharOfLine(); //user input from prompt
bool InitializeOpenNI(); //start openni
void ShutDownOpenNI(); //shutdown openni
bool CreateStreams();// create depth, color video streams
bool CreateDevice();//create openni device
bool CopyAndSave();//copy images to opencv, and save
bool CreateDepth();//create depth frame and copy to opencv (COLOR added)
int SelectResolution(); //not used yet





int main()
{  

//Initialize OpenNI
printf("\r\n---------------------- Init OpenNI --------------------------\r\n");
if (!InitializeOpenNI()) return 1;


//Create and open the device
printf("\r\n---------------------- Create Device --------------------------\r\n");
if (!CreateDevice()) return 2;



//Create the streams
printf("\r\n---------------------- Create Streams --------------------------\r\n");
if (!CreateStreams()) return 3;


//Display the streams
printf("\r\n---------------------- ShowStreams --------------------------\r\n");
char endLoop = ' '; //character to end collection
bool depthOK = false; //check if depth stream was gotten ok.
bool colorOK = false; //check if color stream was gotten ok;
std::cout << "RUNNING DEPTH AND COLOR \r\n" << std::endl;
while (endLoop != 27)//loop until escape is pressed
{
	colorOK = CreateDepth(); //Get the depth stream
	std::cout << colorOK << std::endl; //display a 1 if gotten ok
	endLoop = cv::waitKey(10);//wait for user response on opencv window
};



//shutdown
ShutDownOpenNI();

return 0;
}




/********************************************/
//Error checking
//Displays error if something happens with Kinect
/*******************************************/
bool HandleStatus(openni::Status status)
{
	//initialization fine
	if (status == openni::STATUS_OK)
		return true;
	//if not good, inform user and output error
	printf("Error: #%d, %s", status, openni::OpenNI::getExtendedError());
	//Check for user input
	ReadLastCharOfLine();
	//return indicator to computer
	return false;
}


/********************************************/
//User Input
//Reads the last char typed by user before Enter is pressed
/********************************************/
char ReadLastCharOfLine()
{
	//Variables to read from command line
	int newChar = 0;
	int lastChar;
	//clean up command line
	fflush(stdout);
	//Read all chars from command line until we hit end of line
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n') && (newChar != EOF));
	//Return the last char entered
	return (char)lastChar;
}



/**************************************************/
//Initialize OpenNI
//Initialize all available sensor drivers and scan system for available devices
//This function is called before any other call to kinect
/***************************************************/
bool InitializeOpenNI()
{
	//Print out OpenNI version info
	printf("OpenNI Version is %d.%d.%d.%d", openni::OpenNI::getVersion().major,
		openni::OpenNI::getVersion().minor,
		openni::OpenNI::getVersion().maintenance,
		openni::OpenNI::getVersion().build);

	//Print out info about drivers, and move to next line
	printf("\r\n---------------------- Init OpenNI --------------------------\r\n");
	printf("Checking for devices and loading" " modules/drivers ...\r\n");

	//initialize flag status to ok
	openni::Status status = openni::STATUS_OK;

	//initialize the drivers, and set status to outcome
	status = openni::OpenNI::initialize();

	//Check status
	if (!HandleStatus(status))
	{
		//if not good, inform user and output error
		//return indicator to computer
		return false;
	}

	//Print out all devices connected
	openni::Array<openni::DeviceInfo> deviceList;//openni array for all devices connected
	openni::OpenNI::enumerateDevices(&deviceList);//get the devices connected and put into array
	int numOfDevices = deviceList.getSize(); //get the # of devices found--Should be one usually
	if (numOfDevices > 0)//if we found at least 1
	{
		//Print out # of device available
		printf("%d Devices are available to use \r\n\r\n", numOfDevices);
		//print out their information
		for (int i = 0; i < numOfDevices; i++)
		{
			openni::DeviceInfo device = deviceList[i];
			printf("%d. %s->%s (VID: %d | PID: %d) is connected "
				" at %s\r\n",
				i,
				device.getVendor(),
				device.getName(),
				device.getUsbVendorId(),
				device.getUsbProductId(),
				device.getUri());
		}
	}
	else //print out to user that we find no devices
	{
		printf("+++++++++++++ERROR::NO DEVICES CONNECTED++++++++++++++++++++++++++");
	}

	ReadLastCharOfLine();
	printf("Completed fine. No problems.\r\n");
	return true;

}


/***************************************************/
//Shut down openni
//We are done with kinect, so shut it down please
/******************************************************/
void ShutDownOpenNI()
{
	//Shutdown openni components
	depthFrame.release();
	colorFrame.release();
	depth.stop();
	depth.destroy();
	color.stop();
	color.destroy();

	if (m_streams != NULL)
	{
		delete[]m_streams;
	}
	
	openni::OpenNI::shutdown();

	device.close();
	exit(0);//close application
}


bool CreateDevice()
{
	//initialize flag status to ok
	openni::Status status = openni::STATUS_OK;

	//Attempt device openning
	status = device.open(openni::ANY_DEVICE);//I expect one. Can put in taking from multiple later

	//Check status
	if (status != openni::STATUS_OK)
	{
		printf("NO device created \r \n");
		openni::OpenNI::shutdown();//close device
		return false;
	}

	printf("DEVICE created \r \n");
	return true;
}

bool CreateStreams()
{
	//initialize flag status to ok
	openni::Status status = openni::STATUS_OK;

	//Create depth stream
	status = depth.create(device, openni::SENSOR_DEPTH);

	//Check status
	if (status == openni::STATUS_OK)
	{
		//if ok so far, then open stream
		status = depth.start();
		//Check status
		if (status != openni::STATUS_OK)
		{
			//if not ok, then display extended error from openni
			printf("NO depth created because %s \r \n", openni::OpenNI::getExtendedError());
			depth.destroy(); //close depth stream
			return false;
		}
	}
	else
	{
		//not able to even create the depth
		printf("No depth found because %s \r \n", openni::OpenNI::getExtendedError());
		depth.destroy();
		return false;
	}

	printf("DEPTH created \r \n");
	ReadLastCharOfLine();

	/*******************************************************************/
	//Create color stream
	status = color.create(device, openni::SENSOR_COLOR);

	//Check status
	if (status == openni::STATUS_OK)
	{
		//if ok so far, then open stream
		status = color.start();
		//Check status
		if (status != openni::STATUS_OK)
		{
			//if not ok, then display extended error from openni
			printf("NO color created because %s \r \n", openni::OpenNI::getExtendedError());
			color.destroy(); //close color stream
			return false;
		}
	}
	else
	{
		//not able to even create the depth
		printf("No color found because %s \r \n", openni::OpenNI::getExtendedError());
		color.destroy();
		return false;
	}

	printf("COLOR created \r \n");

	//Synchronize color and depth
	device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	//Store streams
	m_streams = new openni::VideoStream*[NUM_STREAMS];
	m_streams[0] = &depth;
	m_streams[1] = &color;

	//Wait for user input
	ReadLastCharOfLine();
	return true;
}

int SelectResolution(){

	openni::Status status = openni::STATUS_OK;

	//Printout available resolutions ---- and select the ones wanted 
	//video modes for depth
	const openni::Array<openni::VideoMode> *supportedVideoModesD =
		&(depth.getSensorInfo().getSupportedVideoModes());

	//video modes for color
	const openni::Array<openni::VideoMode> *supportedVideoModesC =
		&(color.getSensorInfo().getSupportedVideoModes());

	//size of depth video modes
	int numOfVideoModesD = supportedVideoModesD->getSize();
	if (numOfVideoModesD == 0)//if we get no video modes, problem
	{
		printf("No supported video mode available, press ENTER "
			"to exit.\r\n");
		ReadLastCharOfLine();
		return 1;
	}

	//print out depth video modes
	openni::VideoMode vm;
	for (int i = 0; i < numOfVideoModesD; i++)
	{
		vm = (*supportedVideoModesD)[i];
		printf("%c. %dx%d at %dfps with %d format \r\n",
			'a' + i,
			vm.getResolutionX(),
			vm.getResolutionY(),
			vm.getFps(),
			vm.getPixelFormat());
	}
	printf("Completed Depth modes.\r\n");

	//Select the depth video mode wanted
	int selected = 0;
	do
	{
		printf("Select your desired video mode and then press "
			"ENTER to continue.\r\n");
		selected = ReadLastCharOfLine() - 'a';
	} while (selected < 0 || selected >= numOfVideoModesD);

	//Set and printout the selected video mode
	vm = (*supportedVideoModesD)[selected];
	printf("%dx%d at %dfps with %d format selected. "
		"Requesting video mode ... \r\n",
		vm.getResolutionX(),
		vm.getResolutionY(),
		vm.getFps(),
		vm.getPixelFormat());
	status = depth.setVideoMode(vm);
	if (!HandleStatus(status)) return 1;

	//Get the number of color video modes
	int numOfVideoModesC = supportedVideoModesC->getSize();
	if (numOfVideoModesC == 0)//if no color, problem
	{
		printf("No supported color video mode available, press ENTER "
			"to exit.\r\n");
		ReadLastCharOfLine();
		return 1;
	}

	//Printout all the color modes available
	for (int i = 0; i < numOfVideoModesC; i++)
	{
		vm = (*supportedVideoModesC)[i];
		printf("%c. %dx%d at %dfps with %d format \r\n",
			'a' + i,
			vm.getResolutionX(),
			vm.getResolutionY(),
			vm.getFps(),
			vm.getPixelFormat());
	}
	printf("Completed COlor modes.\r\n");




}

bool CreateDepth()
{
	//Create storage for color and depth opencv images
	cv::Mat depthImage;
	cv::Mat colorImage;

	//Create polling agent
	int streamReadyIndex;//stores the index of the stream we care for
	//wait for depth and color to come through
	openni::Status status = openni::OpenNI::waitForAnyStream(m_streams, 1, &streamReadyIndex, 500);

	//If we are successful in polling
	if (status == openni::STATUS_OK && streamReadyIndex == 0)
	{
		//Read from it
		status = depth.readFrame(&depthFrame);
		//if we get a successful read
		if (status == openni::STATUS_OK && depthFrame.isValid())
		{

			//opencv display
			depthImage.create(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1);
			depthImage.data = (uchar*)depthFrame.getData();
			cv::Mat depthNorm;
			cv::normalize(depthImage, depthNorm, 0, 255, CV_MINMAX, CV_8UC1);
			cv::imshow("DepthOpenCV", depthNorm);
			//printf("GOT DEPTH IMAGE \r\n");
		}
		else
		{
			printf("COULD NOT GET DEPTH IMAGE : %s \r \n", openni::OpenNI::getExtendedError());
			return false;
		}
		/************************************************************************/
		//Read for color
		status = color.readFrame(&colorFrame);
		//if we get a successful read
		if (status == openni::STATUS_OK && colorFrame.isValid())
		{
			//opencv display
			colorImage.create(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3);
			colorImage.data = (uchar*)colorFrame.getData();
			cv::cvtColor(colorImage, colorImage, CV_RGB2BGR);
			cv::imshow("ColorOpenCV", colorImage);
			//printf("GOT COLOR IMAGE \r\n");

			//Update image counter
			imgCount++;

			//TO DO:MOVE TO FUNCTION LATER::CopyAndSave();
			if (imgCount%timeStep == 0)//save every 10th image
			{
				//If we get here, then we have color and depth for this iteration
				//create the image name
				std::string filename = imagePrefix.c_str() + std::to_string(imgIndex) + imgEnd;
				//create the compression parameters for jpeg
				std::vector<int> compression_params;
				compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
				compression_params.push_back(95);
				//write the file
				bool imgSaved = cv::imwrite(filename, colorImage, compression_params);
				//if successful, print out result and update index
				if (imgSaved)
				{
					std::cout << filename << std::endl;

					//Attempt to save corresponding depth image
					filename = depthPrefix.c_str() + std::to_string(imgIndex) + depthEnd;
					//Open a file as a writeable binary file
					std::FILE *fp = std::fopen(filename.c_str(), "wb");
					//write data to file if successful
					if (fp != NULL)
					{
						std::fwrite(depthImage.data, 2, window_h*window_w, fp);
						//close the file
						std::fclose(fp);
						//Update imgIndex
						imgIndex++;
					}
					else
					{
						std::cout << "Error writing depth image \r \n" << std::endl;
						return false;
					}
				}
				else
				{
					std::cout << "Error writing color image \r \n" << std::endl;
					return false;
				}
			}
		}
		else
		{
			printf("COULD NOT GET COLOR IMAGE : %s \r \n", openni::OpenNI::getExtendedError());
			return false;
		}

	}
	else
	{
		printf("Could not read from color/depth stream because : %s \r \n", openni::OpenNI::getExtendedError());
		depth.destroy();
		color.destroy();
		return false;
	}

	return true;
}

