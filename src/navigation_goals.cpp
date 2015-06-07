#include <ros/ros.h>

//ROS Navigation Stack
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


//ARNL Navigation Stack
#include "Aria.h"
#include "ArNetworking.h"

//Other ROS packages
#include <speech_recognition_msgs/SpeechRecognitionCandidates.h> //Import speech messages definition
#include <tf/transform_broadcaster.h> //Convert Angles to Quaternions
#include "sound_play/sound_play.h" //Playing text to voice messages
#include <object_recognition_msgs/RecognizedObjectArray.h>  //Object recognition

#include <fstream>	//files
#include <random>	//Random numbers with different distributions   C++11
#include <chrono>	//Precision chronometer   C++11

#include "explorationMethods.cpp"
#include "internalExploration.cpp"

using namespace std; 
using namespace std::chrono;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//static ros::Publisher kinect_pub_;
//static sound_play::SoundClient robot_voice;

string object="none";
string ARNLPlanningStatus="none";
bool objectFound=false;

//Global odometry
double lastX=0;
double lastY=0;
double lastTheta=0;
double transDistance=0;
double rotDistance=0;
//clock_t executionTime=clock();
high_resolution_clock::time_point executionTime = high_resolution_clock::now();

void sleepok(int t, ros::NodeHandle &n) {
  if (n.ok()) sleep(t);
}

void textToSpeech(string speech, unsigned int counter, sound_play::SoundClient &sc, ros::NodeHandle &nh)
{
   sc.say(speech);
   sleepok(counter, nh);        
}

/*
void loadParams() {
  ros::NodeHandle n_("~");
  if (!n_.getParam ("test", test))
    test = "NONE";
  if (!n_.getParam ("localizationInX", localizationInX))
    localizationInX = 0.0;
  
  if (!n_.getParam ("localizationInY", localizationInY))
    localizationInY = 0.0;
}
*/


vector <int> extractOrder(string str, int maxRoom) {
  //string str must follow format objectNumber-orderRoom1,orderRoom2,orderRoom3,etc
  vector <int> A;
  size_t separatorPos = str.find('-',0);
  while (separatorPos!=string::npos && separatorPos<str.length()) {
    size_t separatorPos2 = str.find(',',separatorPos+1);
    if (separatorPos2==string::npos) separatorPos2=str.length();
    string numberSTR=str.substr(separatorPos+1,separatorPos2-(separatorPos+1));
    int number = atoi(numberSTR.c_str());
    if (number < maxRoom ) A.push_back( number );
    separatorPos=separatorPos2;
  }
  return A;
}


void loadEnvironment(vector<locationNode> &roomList, MatVector &distancesTable, string fileName, MatVector &probabilities, vector<string> &rooms, vector<string> &objects) {
  
  ifstream fileStream;
  fileStream.open (fileName.c_str(), fstream::in);  //std::fstream::in | std::fstream::out | std::fstream::app
  if (!fileStream.fail()) {
    ROS_INFO("Loading data from %s", fileName.c_str());
    cout << "Loading data from " << fileName << endl;
    
    //Load the image file name
    string mapFileName;
    fileStream >> mapFileName;
    cout << "Map image file: " << mapFileName << endl;
    
    //Load the number of rooms
    int roomListSize;
    fileStream >> roomListSize;
    cout << "Rooms: " << roomListSize << endl;
    
    //Load information about each room
    for (int i=0; i<roomListSize; i++) {
      locationNode room;
      room.ID=i;
      cout << "Room ID: " << i << endl;
      //Loead the room center
      fileStream >> room.roomCenter.x >> room.roomCenter.y;
      cout << "Room X: " << room.roomCenter.x << "Room Y: " << room.roomCenter.y << endl;
      //Load the room area
      fileStream >> room.area;
      cout << "Room area: " << room.area << endl;
      //Load the number of border vertex for each room
      int borderCount=0;
      fileStream >> borderCount;
      //Load the vertexs
      for (int j=0; j<borderCount; j++) {
	pointType vertexBorder;
	fileStream >> vertexBorder.x >> vertexBorder.y;
	room.border.push_back(vertexBorder);
      }
      roomList.push_back(room);
    }
    
    //Load the distances between rooms
    doubleVector rowMat;
    rowMat.assign(roomListSize,0);
    for (int i=0; i<roomListSize; i++) {
      distancesTable.push_back(rowMat);
    }
    //distancesTable.release();
    //distancesTable = Mat::zeros(roomList.size(), roomList.size(), CV_32F);
    for (int i=0; i<roomListSize; i++) {
      for (int j=0; j<roomListSize; j++) {
	fileStream >> distancesTable[i][j];
	cout << distancesTable[i][j] << " ";
	//fileStream >> distancesTable.at<float>(i,j);
	//cout << distancesTable.at<float>(i,j) << " ";
      }
      cout << endl;
    }
    fileStream.close();
  } else {
    cout << "Error: No data file found." << endl;
  }
  
  //TODO Data directly assigned because they are not save in map creation. Change the map need to change these names 
  roomList[0].roomName="livingroom";
  roomList[0].roomCenter.x = 5596; roomList[0].roomCenter.y = -467;
  roomList[1].roomName="bedroom";
  roomList[1].roomCenter.x = 6011; roomList[1].roomCenter.y = -4111;
  roomList[2].roomName="bathroom";
  roomList[2].roomCenter.x = 4428; roomList[2].roomCenter.y = -4553;
  roomList[3].roomName="kitchen";
  roomList[3].roomCenter.x = 1762; roomList[3].roomCenter.y = -4742;
  roomList[4].roomName="diningroom";
  roomList[4].roomCenter.x = 2037; roomList[4].roomCenter.y = -1686;
  roomList[5].roomName="studio";  //Lobby
  roomList[5].roomCenter.x = 2663; roomList[5].roomCenter.y = 334;
  roomList[6].roomName="garage";  //Corridor
  roomList[6].roomCenter.x = 4295; roomList[6].roomCenter.y = -1193;
  roomList[7].roomName="patio"; //Entrance
  roomList[7].roomCenter.x = 96; roomList[7].roomCenter.y = -9;
  
  //Delete two last for experiments
  roomList.erase( roomList.end() );
  roomList.erase( roomList.end() );
  cout << "Using " << roomList.size() << " rooms" << endl;
  
  int numObjects=72;  // ######################################## Beacuse those are in the file
  const char *objectvector[] = {"apple","shoe","coffee","clothe","laptop","bread","pen","book","bed sheet","cellphone","spoon","fork","glass of water","handbag","food","towel","chair","medicine","tv remote","coke","broom","mop","key","scissor","comb","bible","ac remote","cell charger","tablet","plate","backpack","baby bottle","headphone","nail clipper","jacket","hand cream","inhaler","cosmetic bag","fly swatter","pillow","blanket","milk","shirt","sock","cup","glasses","knife","soap","coat","pumpkin","orange","paddle","ball","dinosaur","bottle","toy car","frying pan","cd","dvd","videogame","toy","potato chips","cracker","cookie","extinguisher","phone","printer","potty","bookshelf","trash","fridge","softener"};
  vector<string> objectsTMP(objectvector, objectvector+numObjects);
  
  int numRooms=10;  // ######################################## Beacuse those are in the file
  const char *roomvector[] = {"kitchen","bedroom","bathroom","livingroom","diningroom","studio","playroom","patio","laundry","garage"};
  vector<string> roomsTMP(roomvector, roomvector+numRooms);

  //Load probabilities
  //Initialize vector nomObjects x numRooms
  MatVector prob;
  doubleVector rowProb;
  rowProb.assign(numRooms,0);
  for (int i=0; i<numObjects; i++) {
    prob.push_back(rowProb);
  }
  
  ifstream inputFileA("/home/izquierdocr/projects/internetQuery/data/experimentQueryProbabilityWeb.txt");
  for (int i=0; i < numObjects; i++) {
    vector <int> A;
    string lineA;
    getline(inputFileA, lineA); //This do not check if the file is incomplete or unstructured
    A=extractOrder(lineA, numRooms);
    //Fixed probabilities according to an exponential distribution with 8 rooms (but there are 10 in the file)
    double exponentialProbabilities[] = { 0.55610, 0.24920, 0.10580, 0.04860, 0.02150, 0.01110, 0.00500, 0.00150, 0, 0 };
    for (int j=0; j < numRooms; j++) {
      prob[i][j] = exponentialProbabilities[ A[j] ];
    }
  }
  inputFileA.close();
  
  rooms=roomsTMP;
  objects=objectsTMP;
  probabilities=prob;
}


void estimateProbabilities(vector<locationNode> &locationList, string object, MatVector probabilities, vector<string> rooms, vector<string> objects) {
  
  int objectPosition=-1;
  for (int i=0; i<objects.size(); i++) {
    if ( object==objects[i] ) {
      objectPosition=i;
      break;
    }
  }
  if ( objectPosition<0) {
    cout << "Object not found in DB" << endl;
    exit(0);
    //Buscarlo en Internet, no en el archivo;
  }
  
  for (int i=0; i<locationList.size(); i++) {
    int roomPosition=-1;
    for (int j=0; j<rooms.size(); j++) {
      if ( locationList[i].roomName==rooms[j] ) {
	roomPosition=j;
	break;
      }
    } 
    if ( roomPosition<0) {
      cout << "Room not found in DB" << endl;
      exit(0);
      //Buscarlo en Internet, no en el archivo;
    } 
    
    locationList[i].objectProbability=probabilities[objectPosition][roomPosition];
    //locationList[i].objectProbability=(double (rand() ))/RAND_MAX;  //random probabilities
  }
}


geometry_msgs::Quaternion angle2quaternion(double theta) {
  //double grades //Counter-clock angle in grades
  double radians = theta * (M_PI/180);

  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quaternion, qMsg);
  return qMsg;
}

//void setGoal(MoveBaseClient ac, double x, double y, double theta, std::string frameID) {
bool setGoalROS(double x, double y, double theta, std::string frameID) {
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frameID; //map:world relative, base_link: robot relative
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  if ( frameID=="map" )
    goal.target_pose.pose.orientation = angle2quaternion(theta);
  else
    goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal reached");
    return true;
  }
  else {
    ROS_INFO("The robot failed to move to the goal for some reason");
    return false;
  }
}

bool setGoalARNL(ArClientBase *client, double x, double y, double theta=-1) {
  ArNetPacket posePacket;
  posePacket.byte4ToBuf( int (x) );
  posePacket.byte4ToBuf( int (y) );
  if (theta>=0) posePacket.byte4ToBuf( int (theta) );
  client->requestOnce( "gotoPose", &posePacket );
  
  //TODO If the goal is not reached this is an infinite cicle because error is not handled
  while ( ARNLPlanningStatus!="" ) {
    client->requestOnce("pathPlannerStatus");
    client->requestOnce("update");
    if (ARNLPlanningStatus.substr(0,6)=="Failed") {
      ARNLPlanningStatus="none";
      return false;
    }
    sleep(1);
  }
  ARNLPlanningStatus="none";
  return true;
}


string getNameFromID(string ID) {
  
  if (ID == "3312175a69e13571594fa93843016063") return "Aceitunas";
  if (ID == "3312175a69e13571594fa9384302c4ae") return "Arizona";
  if (ID == "3312175a69e13571594fa93843040d2c") return "Coca";
  if (ID == "3312175a69e13571594fa93843055ce7") return "Salsa Inglesa";
  if (ID == "3312175a69e13571594fa93843068b5b") return "Mayonesa";
  if (ID == "3312175a69e13571594fa9384307ba15") return "Mermelada";
  if (ID == "3312175a69e13571594fa9384308f031") return "Pure";
  if (ID == "3312175a69e13571594fa938430a4f70") return "Valentina";
  if (ID == "3312175a69e13571594fa938430b7c0b") return "Sangria";
  if (ID == "3312175a69e13571594fa938430c9255") return "Zucaritas";
  return "Unknow Object";
}



void object_detector(const  object_recognition_msgs::RecognizedObjectArray& msg) {
  int objects_detected = msg.objects.size();
  bool detected_object_status=true;
  int imageObjectDetectedNSec=msg.header.stamp.toNSec();
  vector<string> object_id;
  
  if (objects_detected > 0) {
    ROS_INFO("%d objects detected",objects_detected);
    ROS_INFO("Recognizer Header seq: %d",msg.header.seq);
    ROS_INFO("Recognizer Header time stamp secs: %f",msg.header.stamp.toSec());
    ROS_INFO("Recognizer Header time stamp nsecs: %ld",msg.header.stamp.toNSec());
    ROS_INFO("Recognizer Header frame: %s",msg.header.frame_id.c_str());

    for (int i=0; i<objects_detected; i++) {	
      object_id.push_back( msg.objects[i].type.key );
      ROS_INFO("Object key: %s",object_id[i].c_str());
      ROS_INFO("Confidence: %f",msg.objects[i].confidence);
      //int bd=msg.objects[i].bounding_contours.size();
      //ROS_INFO("Bounding contours: %d",bd);
      ROS_INFO("Position X: %f",msg.objects[i].pose.pose.pose.position.x);
      ROS_INFO("Position Y: %f",msg.objects[i].pose.pose.pose.position.y);
      ROS_INFO("Position Z: %f",msg.objects[i].pose.pose.pose.position.z);
      if ( getNameFromID(object_id[i]) == object ) objectFound=true;
    }
  }
  else detected_object_status=false;
}


void voice(const speech_recognition_msgs::SpeechRecognitionCandidates& commandList) {
  object = commandList.transcript[0];
  ROS_INFO ("Best match: %s", object.c_str() );

  //place=command; //Need to be filtered/analized to match a valid command

  //ROS_INFO ("Confidence: %f", commandList.confidence[0] ); //Android app is not sending confidence
}

void handleGoalList(ArNetPacket *packet) {
  printf(".. Server has these goals:\n");  
  char goal[256];   
  for(int i = 0; packet->getReadLength() < packet->getLength(); i++)   
  {   
    packet->bufToStr(goal, 255);   
    if(strlen(goal) == 0)   
      return;   
    printf("      %s\n", goal);   
  }   
}   


void handlePathPlannerStatus(ArNetPacket *packet)
{
  char buf[64];
  packet->bufToStr(buf, 63);
  string plannerStatus=buf;
  cout << "Path planner status: " << plannerStatus << endl;
  
  ARNLPlanningStatus=plannerStatus;
}


void handleRobotUpdate(ArNetPacket* packet) {
  char buf[64];
 
  packet -> bufToStr(buf, 63);
  string serverStatus=buf;
  packet -> bufToStr(buf, 63);
  string serverMode = buf;
  double battery = packet -> bufToByte2();
  double x = packet -> bufToByte4();
  double y = packet -> bufToByte4();
  double theta = packet -> bufToByte2();
  double transVelocity = packet -> bufToByte2();
  double rotVelocity =  packet -> bufToByte2();
  
  //cout << "Robot server status: " << serverStatus << endl;
  //cout << "Robot position: (" << x << "," << y << "," << theta << ")" << endl;
  transDistance += sqrt( pow(x-lastX,2) + pow(y-lastY,2) );
  rotDistance += abs(lastTheta - theta );
  lastX = x; lastY = y; lastTheta = theta;
  cout << "Traveled distance: " << transDistance << endl;
  cout << "Turns made: " << rotDistance << endl;
  duration<double> time_span = duration_cast<duration<double>>(high_resolution_clock::now() - executionTime);
  cout << "Execution time: " << time_span.count() << " seconds" << endl;
  //cout << "Execution time: " << ((double)(clock()-executionTime))/CLOCKS_PER_SEC << endl;
}

void resetOdometry(ArClientBase *client) {
  client->requestOnce("update");
  transDistance = 0;
  rotDistance = 0;
  executionTime = high_resolution_clock::now();
}


void generateRoomProbabilities(string distributionType, int nintervals, vector<double> &p){
  
  const bool showGraph=true;              //Show a graph with *
  const int experiments=10000;             // number of experiments
  const int starsToShow=10*nintervals;     // maximum number of stars to distribute in the graph
  
  default_random_engine generator;
  
  uniform_real_distribution<double> distributionU(0.0,1.0);
  normal_distribution<double> distributionN(0.5,0.2);
  gamma_distribution<double> distributionG(2.0,0.2);
  exponential_distribution<double> distributionE(6.5);
  
  int notused=0;
  p.assign(nintervals,0);
  
  for (int i=0; i<experiments; ++i) {
    double number;
    if (distributionType=="Uniform") number = distributionU(generator);
    if (distributionType=="Normal") number = distributionN(generator);
    if (distributionType=="Gamma") number = distributionG(generator);
    if (distributionType=="Exponential") number = distributionE(generator);
    //cout << number << endl;
    if (number>=0 && number<1) {
      ++p[int(nintervals*number)];
    }
    else
      notused++;
  }
  
  cout << "Using distribution " << distributionType << endl;
  cout << "Percentage unused random numbers (out of range):" << double(100*notused)/experiments << endl << endl;
  
  cout << fixed; cout.precision(5);
  double totalP=0;
  for (int i=0; i<nintervals; ++i) {
    if (showGraph) {
      cout << double(i)/nintervals << "-" << double(i+1)/nintervals << ": ";
      cout << string(int(p[i])*starsToShow/experiments,'*');
      cout << "   " << p[i]/experiments << endl;
    }
    p[i]/=experiments;
    totalP+=p[i];
  }
  //Normalize to 1
  for (int i=0; i<nintervals; ++i) p[i]/totalP;
}


void internalSearch(ArClientBase *client, string roomName, double xInit, double yInit){
  double minX,minY;
  double maxX,maxY;
  double resolution;
  
  Mat map;
  loadMapfromAria(map, minX, minY, maxX, maxY, resolution, "/home/izquierdocr/maps/lab_2015.map");
  
  Mat mapFree;
  Mat kernel = Mat::ones(14,14,CV_8UC1);
  dilate(map, mapFree, kernel, Point(-1,-1), 1);
  
  vector <boostPolygonType> polygonsFile;  //Load polygon reads many polygons from a file but we only have one for cone
  loadPolygons(polygonsFile, "/home/izquierdocr/catkin_ws/src/object_search/config/visibilityCone.txt");
  boostPolygonType visibilityCone = polygonsFile[0];
  
  vector <boostPolygonType> flatSurfaces;
  loadPolygons(flatSurfaces, "/home/izquierdocr/catkin_ws/src/object_search/config/flatSurfaces.txt");
  drawFlatSurfaces(mapFree, flatSurfaces, minX, minY, maxX, maxY, resolution);
  
  vector <boostPolygonType> room;
  loadPolygons(room, "/home/izquierdocr/catkin_ws/src/object_search/config/"+roomName+".txt");
  
  vector <boostPolygonType> flatSurfacesinRoom;
  flatSurfacesinRoom = selectFlatSurfaces(room[0], flatSurfaces);
  
  poseArrayType poses;
  
  int numPoses=2000;
  generatePoses(mapFree, poses, numPoses, visibilityCone, room[0], minX, minY, maxX, maxY, resolution);
  
  //int maxPoses=100;
  int maxPoses=numPoses*0.10;
  evaluatePoses(poses, flatSurfacesinRoom, visibilityCone, maxPoses);
  
  double obstacleDistance=300;  //Distance to be considered close to obstacles in milimeters
  filterCloseToObstacles(mapFree, poses, obstacleDistance, minX, minY, maxX, maxY, resolution);
  
  double closeDistance=1000;  //Distance to be considered close to other poses in milimeters
  double closeAngle=30;   //Angle to be considered close to other poses in degrees
  filterSimilarPoses(poses, closeDistance, closeAngle);
  
  cout << poses.size() << " poses to be used" << endl;
  //drawPoses(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution);
  
  selectPath(poses, xInit, yInit);
  //drawPath(mapFree, poses, visibilityCone, minX, minY, maxX, maxY, resolution);
  
  for (int i=0; i<poses.size(); i++) {
    double thetaForRobot=fmod(450-poses[i].theta,360); //Adjusting for counterclockwise and zero displacement for robot
    setGoalARNL(client, poses[i].x, poses[i].y, thetaForRobot);
    
    cout << "Visually searching object at pose (" << poses[i].x << "," << poses[i].y << "," << poses[i].theta << ")" << endl;
    
    //drawOnePose(mapFree, poses[i].x , poses[i].y , poses[i].theta, visibilityCone, minX, minY, maxX, maxY, resolution);
    if (objectFound) return;
  }
  
}


int main(int argc, char** argv){
  
  //Print probability values from distribtion
  //vector<double> roomProbabilities;
  //generateRoomProbabilities("Exponential", 8, roomProbabilities);
  //return 0;
  
  /* 
  //ARNL Initialization
  int argsNumber=3;
  char *arguments[argsNumber];
  arguments[0]="./simpleGoals";
  arguments[1]="-host";
  arguments[2]="10.41.42.1";
  
  Aria::init(); 
  ArClientBase client;
  ArArgumentParser parser(&argsNumber, arguments);
  ArClientSimpleConnector clientConnector(&parser);
  parser.loadDefaultArguments();
  */
  
  
  Aria::init(); 
  ArClientBase client;
  ArArgumentParser parser(&argc, argv);
  ArClientSimpleConnector clientConnector(&parser);
  parser.loadDefaultArguments();
  
    if (!clientConnector.connectClient(&client)) {
    if (client.wasRejected())
      cout << "Server rejected connection, exiting" << endl;
    else
      cout << "Could not connect to server, exiting" << endl;
    
    exit(1);
  } 
  
  //Set ARNL Handlers. There is not predefined planner and robot status handler
  //client.addHandler("goalList", new  ArGlobalFunctor1<ArNetPacket*>(&handleGoalList));
  client.addHandler("pathPlannerStatus", new ArGlobalFunctor1<ArNetPacket*>(&handlePathPlannerStatus));
  client.addHandler("update", new ArGlobalFunctor1<ArNetPacket*>(&handleRobotUpdate));
  
  client.runAsync();   
  //client.requestOnce("goalList");  
  //client.requestOnceWithString( "gotoGoal", "living" );

  //ROS Initialization
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  //Suscribe to android voice recognizer node
  ros::Subscriber voice_sub_= nh.subscribe("/voice", 1, voice);

  //And get recognized objects from ORK (Object Recognition Kitchen)
  ros::Subscriber object_array_sub_= nh.subscribe("/recognized_object_array", 1, object_detector);
  
  //Create a voice synthesizer device
  sound_play::SoundClient robot_voice;
  sleepok(1, nh);

  
  vector<locationNode> locationList;
  MatVector distancesTable;
  int startingNode;
  
  
  MatVector probabilities;
  vector<string> rooms;
  vector<string> objects;
  loadEnvironment(locationList, distancesTable, "/home/izquierdocr/catkin_ws/src/object_search/config/mapRoboticLabINAOE.txt", probabilities, rooms, objects);
  
  cout << "Entering cycle" << endl;
  
  
  object="blanket";  //TODO%%%%%%%%%%%%%%%%%%%%$$$$$$$$$$$$$$$$$$$$########################

  while (ros::ok()) {
    objectFound=false;
    robot_voice.say("I am listening you!. Please tell me an object.","");
    sleepok(8, nh);  
    
    
    cout << "Waiting for object to search for" << endl;
    while (object=="none") {ros::spinOnce();} //Wait for an object from user topic /voice
    
    //clock_t initTime;
    //initTime=clock();
    high_resolution_clock::time_point initTime = high_resolution_clock::now();
    resetOdometry(&client);
    
    cout << "Estimating probabilities" << endl;
    estimateProbabilities(locationList, object, probabilities, rooms, objects);
    object="none";
    
    cout << "Generating exploration order" << endl;
    locationNode presentPose; presentPose.roomCenter.x = lastX; presentPose.roomCenter.y = lastY;
    locationList.insert(locationList.begin(), presentPose);
    startingNode=0;
    generateRouteExpectedDistanceSimple(locationList, distancesTable, startingNode);
    cout << "Order: ";
    for (int i=0; i<locationList.size(); i++) {
      cout << locationList[i].roomName << ",";
    }
    cout << endl;
    
    cout << "Exploring" << endl;
    for (int i=1; i<locationList.size(); i++) {
      
      cout << "Room priority: " << i << endl;
      cout << "Room: " << locationList[i].roomName << endl;
      cout << "Pos X: " << locationList[i].roomCenter.x << endl;
      cout << "Pos Y: " << locationList[i].roomCenter.y << endl;
      cout << "Pos Area: " << locationList[i].area << endl;
      cout << "Probability: " << locationList[i].objectProbability << endl;
      
      
      string voiceMsg="Going to search in the " + locationList[i].roomName;
      robot_voice.say(voiceMsg.c_str(),"");
      
      //if ( setGoalROS(locationList[i].roomCenter.x, locationList[i].roomCenter.y, 0, "map") ) voiceMsg = "I am in the " + locationList[i].roomName + " right now";
      if ( setGoalARNL(&client, locationList[i].roomCenter.x, locationList[i].roomCenter.y) ) {
	voiceMsg = "I am in the " + locationList[i].roomName + " right now";
	internalSearch( &client, locationList[i].roomName, locationList[i].roomCenter.x, locationList[i].roomCenter.y );
      }
      else {
	voiceMsg = "I could not reach the " + locationList[i].roomName;
      }
      robot_voice.say(voiceMsg.c_str(),"");
      if (objectFound) {
	duration<double> time_span = duration_cast<duration<double>>(high_resolution_clock::now() - initTime);
	cout << "Objet found in " << locationList[i].roomName << " in " << time_span.count() << " seconds" << endl;
	//cout << "Objet found in " << locationList[i].roomName << " in " << ((double)(clock()-initTime))/CLOCKS_PER_SEC << " seconds" << endl;
	cout << "Traveled distance " << transDistance << " with " << rotDistance << " degrees turned" << endl;
	break;
      }
    }
    
    if (!objectFound) {
      duration<double> time_span = duration_cast<duration<double>>(high_resolution_clock::now() - initTime);
      cout << "Objet not found. Searching last " << time_span.count() << " seconds" << endl;
      //cout << "Objet not found. Searching last " << ((double)(clock()-initTime))/CLOCKS_PER_SEC << " seconds" << endl;
      cout << "Traveled distance " << transDistance << " with " << rotDistance << " degrees turned" << endl;
    }


    ros::spinOnce();
    //loop_rate.sleep();
  }
  
  //ros::spin();
  Aria::shutdown();  
  
  return 0;
}


/*



if (command=="kitchen") {
  string textToSay="Going to the kitchen now";
  robot_voice.say(textToSay.c_str(),"");
	if ( setGoal(7.5, -7.5, 90, "map") ) robot_voice.say("Goal reached","");
			command="none";
	}
	else if (command=="bedroom") {
	  string textToSay="Going to the bedroom now";
	  robot_voice.say(textToSay.c_str(),"");
	if ( setGoal(-3.5, -2.0, 270, "map") ) robot_voice.say("Goal reached","");
			command="none";
	}
	else if (command=="entrance") {
	  string textToSay="Going to the entrance now";
	  robot_voice.say(textToSay.c_str(),"");
	if ( setGoal(-11.5, -17.5, 270, "map") ) robot_voice.say("Goal reached","");
			command="none";
	}
	else if (command=="forward") {
	  string textToSay="Moving one meter forward";
	  robot_voice.say(textToSay.c_str(),"");
	if ( setGoal(1.0, 0.0, 1.0, "base_link") ) robot_voice.say("Goal reached","");
			command="none";
	}
	else if (command=="backward") {
	  string textToSay="Moving one meter backward";
	  robot_voice.say(textToSay.c_str(),"");
	if ( setGoal(-1.0, 0.0, 1.0, "base_link") ) robot_voice.say("Goal reached","");
			command="none";
	}
	else if (command=="none") {
	  //Do nothing
	  }
	  else {
	    string textToSay="I could not understand the command";
	    robot_voice.say(textToSay.c_str(),"");
	//sleepok(8, n);
	command="none";
	}







vector<locationNode> locationList;
MatVector distancesTable;

buildEnvironment(locationList, distancesTable, numNodes, environmentType, probabilityDistribution);

optimalDistance=maxDistance(distancesTable,locationList)*(numNodes+1);
optimalExpectedDistance=optimalDistance;
optimalExpectedDistanceMurrieta=optimalDistance; //Para el metodo del Dr. Murrieta
maxDistanceUCT=optimalDistance; //Para standarizar en [0,1] las distancias
worstDistance=0;
worstExpectedDistance=0;
optimalHeuristicDistance=0;
partialOptimalHeuristicDistance=0;

cout << endl << "Building strategies..." << endl;
vector<locationNode> locationListOptimal, locationListWorst, locationListOptimalExpected, locationListWorstExpected, locationListOptimalHeuristic, locationListMostProbable, locationListClosest, locationListRandom, locationListUCT, locationListExpectedDistanceFixed, locationListExpectedDistanceMurrieta, locationListExpectedDistanceSimple, locationListPartialOptimalHeuristic;
locationListOptimal=locationList;
locationListWorst=locationList;
locationListOptimalExpected=locationList;
locationListWorstExpected=locationList;
locationListOptimalHeuristic=locationList;
locationListMostProbable=locationList;
locationListClosest=locationList;
locationListRandom=locationList;
locationListUCT=locationList;
locationListExpectedDistanceFixed=locationList;
locationListExpectedDistanceMurrieta=locationList;
locationListExpectedDistanceSimple=locationList;
locationListPartialOptimalHeuristic=locationList;


clock_t initTime;
initTime=clock();
generateRouteOptimalAndWorst(locationListOptimal, locationListWorst, locationListOptimalExpected, locationListWorstExpected, locationListOptimalHeuristic ,distancesTable, startingNode);

generateRouteMostProbable(locationListMostProbable, distancesTable, startingNode);
generateRouteClosest(locationListClosest, distancesTable, startingNode);
generateRouteRandom(locationListRandom, distancesTable, startingNode);
generateRouteUCT(locationListUCT, distancesTable, startingNode);
generateRouteExpectedDistanceFixed(locationListExpectedDistanceFixed, distancesTable, startingNode);
generateRouteExpectedDistanceMurrieta(locationListExpectedDistanceMurrieta, distancesTable, startingNode);
generateRouteExpectedDistanceSimple(locationListExpectedDistanceSimple, distancesTable, startingNode);
generateRoutePartialOptimalHeuristic(locationListPartialOptimalHeuristic, distancesTable, startingNode);


*/