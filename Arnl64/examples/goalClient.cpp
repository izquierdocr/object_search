/*

Copyright (c) 2014 Adept Technology Inc.
All rights reserved.  

Redistribution of this example source code, with or without modification, is 
permitted provided that the following conditions are met:   
-    Redistributions must retain the above copyright notice, 
     this list of conditions and the following disclaimer.  
-    Redistributions must be in source code form only

The information in this document is subject to change without notice and should
not be construed as a commitment by Adept Technology, Inc.

Adept Technology, Inc. makes no warranty as to the suitability of this material
for use by the recipient, and assumes no responsibility for any consequences
resulting from such use. 

Note: All other non-example software, including binary software objects
(libraries, programs), are prohibited from distribution under terms described
in LICENSE.txt (refer to LICENSE.txt for details).
*/


#include "Aria.h"
#include "ArNetworking.h"



void handlePathPlannerStatus(ArNetPacket *packet)
{
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Path planner status: \"%s\"\n", buf);
}


void handleGoalName(ArNetPacket* packet)
{
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Current goal: \"%s\"\n", buf);
}

void handleRobotUpdate(ArNetPacket* packet)
{
  char buf[64];
  packet->bufToStr(buf, 63);
  printf(".. Robot server status: \"%s\"\n", buf);
  packet->bufToStr(buf, 63);
  printf(".. Robot server mode: \"%s\"\n", buf);
}


void handleGoalList(ArNetPacket *packet)
{
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
    
int main(int argc, char **argv)
{
  Aria::init();
  ArClientBase client;
  ArArgumentParser parser(&argc, argv);
  ArClientSimpleConnector clientConnector(&parser);
  parser.loadDefaultArguments();

  if (!clientConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {
    clientConnector.logOptions();
    exit(0);
  }
  
  printf("Connecting...\n");
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      printf("Server rejected connection, exiting\n");
    else
      printf("Could not connect to server, exiting\n");
    
    exit(1);
  } 

  printf("Connected to server.\n");
  client.addHandler("pathPlannerStatus", new ArGlobalFunctor1<ArNetPacket*>(&handlePathPlannerStatus));
  client.addHandler("update", new ArGlobalFunctor1<ArNetPacket*>(&handleRobotUpdate));
  client.addHandler("goalName", new ArGlobalFunctor1<ArNetPacket*>(&handleGoalName));
  client.addHandler("getGoals", new  ArGlobalFunctor1<ArNetPacket*>(&handleGoalList));
  client.runAsync();
  client.requestOnce("getGoals");
  client.request("pathPlannerStatus", 5000);
  client.request("goalName", 5000);
  client.request("update", 5000);
  while(client.getRunningWithLock())
  {
    char goal[128];
    printf("=> Enter a goal name, or * to tour all goals, or ? to list goals.\n");
    if( fgets(goal, 127, stdin) == NULL )
    {
        // EOF
        printf("goodbye.\n");
        Aria::shutdown();
        return 0;
    }
    goal[strlen(goal)-1] = '\0';  // remove newline
    if(strcmp(goal, "?") == 0)
    {
      client.requestOnce("getGoals");
      printf("=> Enter a goal name, or * to tour all goals, or ? to list goals.\n");
    }
    else if(strcmp(goal, "*") == 0)
    {
      if(client.dataExists("tourGoals"))
      {
        printf("=> Touring all goals...\n");
        client.requestOnce("tourGoals");
      }
      else
      {
        printf("=> Can't tour goals, server does not have that request.\n");
      }
    }
    else
    {
      printf("=> Sending goal \"%s\" to server...\n", goal);
      client.requestOnceWithString("gotoGoal", goal);
    }
  }
  printf("Server disconnected.\n");
  Aria::shutdown();
  return 0;
}
