/*
 * Software License Agreement (GPLv2 License)
 * 
 * Copyright (c) 2011 OpenQbo, Inc.
 *
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 * Author: Arturo Bajuelos <arturo@openqbo.com>
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <sstream>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h>
#include <stdio.h>
#include <cxcore.h>
#include <climits>

#include "Orbit.h"

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <qbo_talk/Text2Speach.h>

#include <qbo_listen/Listened.h>

#include <std_msgs/String.h>

#include "qbo_object_recognition/LearnNewObject.h"
#include "qbo_object_recognition/RecognizeObject.h"
#include "qbo_object_recognition/Teach.h"
#include "qbo_object_recognition/Update.h"

#include <qbo_self_recognizer/QboRecognize.h>
#include <boost/algorithm/string.hpp>
#include "boost/filesystem.hpp"


using namespace std;

ros::Subscriber listener_sub;

ros::ServiceClient client_talker;
ros::ServiceClient client_self_recognizer;

qbo_talk::Text2Speach srv_talker;
qbo_self_recognizer::QboRecognize srv_self_recognize;

string recognized_object = "";

ros::ServiceClient client_learn;
ros::ServiceClient client_recognize;
ros::ServiceClient client_teach;
ros::ServiceClient client_update;

qbo_object_recognition::LearnNewObject srv_learn;
qbo_object_recognition::RecognizeObject srv_recognize;
qbo_object_recognition::Teach srv_teach;
qbo_object_recognition::Update srv_update;


ros::Time last_object_received_;


bool learn_request = false;
string object_to_learn = "";

string objects_path = "/opt/ros/electric/stacks/qbo_stack/qbo_object_recognition/objects/objects_db/";

void speak_this(string to_speak)
{
	srv_talker.request.command = to_speak;

	if (client_talker.call(srv_talker))
		ROS_INFO("Talked: %s", to_speak.c_str());
	else
		ROS_ERROR("Failed to call the service of qbo_talk");
}


void stereoSelectorCallback(const sensor_msgs::Image::ConstPtr& image_ptr)
{
	last_object_received_ = ros::Time::now();
}

bool self_recognize()
{

	if (client_self_recognizer.call(srv_self_recognize))
		return srv_self_recognize.response.recognized;
	else
		ROS_ERROR("Failed to call the service of qbo_self_recognizer");
		
	return false;
}

bool learnAndTeach(string object_name)
{
	srv_learn.request.object_name = object_name;

	if (client_learn.call(srv_learn))
	{
		if(srv_learn.response.learned)
			ROS_INFO("Images captured successfully!");
	}
	else
	{
		ROS_ERROR("Failed to call service learn object");
	}

	if(object_name == "MYSELF")
		speak_this("Now I get the picture of how I look. I will train myself.");
	else
		speak_this("Now I get the picture of "+object_name+". I will train myself.");

	if (client_teach.call(srv_teach))
	{
		if(srv_teach.response.taught)
			ROS_INFO("Orbit taught successfully!");
	}
	else
	{
		ROS_ERROR("Failed to call service teach object");
	}


	return srv_learn.response.learned;
}

bool forget_object(string object_name)
{
	speak_this("Forgetting object. "+object_name+".");

	string object_to_forget_path = objects_path+"/"+object_name;

	if(!boost::filesystem::is_directory(object_to_forget_path))
	{
		speak_this("Sorry but cannot forget the object. "+object_name+" . I don't know how a "+object_name+" looks like.");
		return true;
	}

	else
	{

		boost::filesystem::remove_all(object_to_forget_path);

		if(boost::filesystem::is_regular_file(objects_path+"/vocabulary.xml.gz"))
			boost::filesystem::remove(objects_path+"/vocabulary.xml.gz");

		speak_this("I am re training myself");

		if (client_update.call(srv_update))
		{
			if(srv_update.response.updated)
				ROS_INFO("Orbit updated successfully!");
		}
		else
		{
			ROS_ERROR("Failed to call service update orbit");
			return false;
		}

		speak_this("OK. I have forgotten object "+object_name);
	}

	return true;
}

string recognize()
{
	string name_detected = "";
	//Use the service
	if (client_recognize.call(srv_recognize))
	{
		if(srv_recognize.response.recognized)
			name_detected = (std::string)(srv_recognize.response.object_name);
	}
	else
	{
		ROS_ERROR("Failed to call service recognize object");
	}

	return name_detected;
}

void listenerCallback(const qbo_listen::ListenedConstPtr& msg)
{
	ros::Time time_now = ros::Time::now();
	
	ros::Duration  time_diff = time_now - last_object_received_;
	



	std::string listened = msg->msg;

	/*Added for mirror video 2*/
/*	if(listened =="HELLO CUBE E O")
		speak_this("Hello Arturo");
	else if(listened == "LETS MAKE AN EXPERT MEN")
		speak_this("Ok. Lets do it");
*/

	if(listened == "" and msg->not_msg == "WHAT IS THIS")
		listened = "WHAT IS THIS";
	else if(listened == "" and msg->not_msg == "WHAT IS THIS CUBE E O")
		listened = "WHAT IS THIS";
	else if(listened == "" and msg->not_msg == "CUBE E O WHAT IS THIS")
		listened = "WHAT IS THIS";

	else if(listened == "" and msg->not_msg == "CUBE E O THIS IS YOU")
		listened = "CUBE E O THIS IS YOU";

    ROS_INFO("Listened: %s", listened.c_str());



	if(time_diff.toSec()>0.3)
	{
		ROS_INFO("Ignoring last sentence because object is not spotted!!");
		return;
	}

	if(learn_request) //A name has been asked to be learned
	{
		if(string(listened) == "YES I DID") //Confirm the name
		{
			speak_this("I'm learning how "+object_to_learn+" looks like");
			//cv::waitKey(5000);
			learnAndTeach(object_to_learn);
			speak_this("I am ready to recognize "+object_to_learn);

			learn_request = false;
		}
		else if(string(listened) == "NO" || string(listened) == "NO I DID NOT")
		{
			learn_request = false;
			speak_this("Can you repeat the name of the object please");
			//cv::waitKey(5000);
		}
		else
			return;
	}

	else if(string(listened) == "CUBE E O WHAT IS THIS" || string(listened) == "DO YOU KNOW WHAT THIS IS" ||
			string(listened) == "WHAT IS THIS CUBE E O" || string(listened) == "WHAT IS THIS" )
	{

		speak_this("Let me see it");

		//cv::waitKey(3000);

		ROS_INFO("Trying to recognize");
		string recognized_object = recognize();

		if(recognized_object != "")
		{	//speak_this("This is a "+recognized_object);
			if(recognized_object == "PENGUIN")
				speak_this("I got it. I think it is "+recognized_object);
			else if(recognized_object == "MYSELF")
				speak_this("Oh. This is me. Nice");
			else if(recognized_object =="CUBE E O")
			{

				speak_this("Oh. It's a q b o. Cool");
				
				/*
				speak_this("Interesting. It looks like a Q b o. Let me check who he is");
			
				//Kill qbo_stereo_selector node Because of the nose light on
				string command = "rosnode kill /qbo_stereo_selector";
		    		system(command.c_str());
			
				bool it_is_me = self_recognize();
			
				if(it_is_me)
					speak_this("Oh. It is me. I must have a mirror in front of me");
				else
				{	

				//listener_sub.shutdown();
			  	speak_this("Oh. It's another q b o.");
				//command = "rosrun qbo_questions questions2.py &";
		        //system(command.c_str());
				        
		                     //ignore = true;
				}
			//Re launch qbo_stereo_selector	
			command = "roslaunch qbo_stereo_selector qbo_stereo_selector.launch &";
		        system(command.c_str());
			*/
		}
		else
			speak_this("I got it. I think it is a "+recognized_object);

		}
		else
			speak_this("I don't know");
	}
	else if(string(listened) == "WHO IS THIS")
	{
		speak_this("Let me see");

		//cv::waitKey(3000);

		ROS_INFO("Trying to recognize");
		string recognized_object = recognize();


		if(recognized_object == "MYSELF")
			speak_this("Oh. This is me. Nice");
		else if(recognized_object =="CUBE E O")
		{
			speak_this("Oh. It's a q b o. Cool");
		
			/*
				speak_this("Interesting. It looks like a Q b o. Let me check who he is");
			
			
			//Kill qbo_stereo_selector node Because of the nose light on
			string command = "rosnode kill /qbo_stereo_selector";
		    system(command.c_str());
			
			bool it_is_me = self_recognize();
			
			if(it_is_me)
				speak_this("Oh. It is me. I must have a mirror in front of me");
		    else
			{
			//	listener_sub.shutdown();
			    speak_this("Oh. It's another q b o.");
				//command = "rosrun qbo_questions questions2.py &";
		        //system(command.c_str());
		        
		        //ignore = true;
			}
			
			//Re launch qbo_stereo_selector	
			command = "roslaunch qbo_stereo_selector qbo_stereo_selector.launch &";
		        system(command.c_str());
                         
                       */
		}
		else
			speak_this("I don't know");
	}
	else if(string(listened) == "CUBE E O THIS IS YOU" || string(listened) =="THIS IS YOU CUBE E O")
	{
		speak_this("Oh. Let me see how I look.");
		//cv::waitKey(5000);
		learnAndTeach("MYSELF");
		speak_this("Wo aw.. I am ready to recognize myself.");
	}
	else
	{
		vector<string> words;
		boost::split(words, listened, boost::is_any_of(" "));
		if(words.size()>3 && words[0]=="THIS")
		{

			//Erase THIS IS A
			for(unsigned int i = 0; i<3;i++)
			{
				words.erase(words.begin());
			}


			string object_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				object_name+=words[i];

				if(i!=words.size()-1)
					object_name+=" ";
			}

			speak_this("Did you say "+object_name+"?");
			//cv::waitKey(5000);

			learn_request = true;
			object_to_learn = object_name;


		}
		else if(words.size()>4 && words[0]=="CUBE" && words[3] == "THIS") //CUBE E O THIS IS A
		{

			//Erase CUBE E O THIS IS A
			for(unsigned int i = 0; i<6;i++)
			{
				words.erase(words.begin());
			}


			string object_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				object_name+=words[i];

				if(i!=words.size()-1)
					object_name+=" ";
			}

			speak_this("Did you say "+object_name+"?");
			//cv::waitKey(5000);

			learn_request = true;
			object_to_learn = object_name;


		}

		else if(words.size()>3 && words[0]=="PLEASE" && words[1] == "FOR") //PLEASE FOR GET
		{
			//Erase PLEASE FOR GET
			for(unsigned int i = 0; i<3;i++)
			{
				words.erase(words.begin());
			}

			string object_name;

			for(unsigned int i = 0; i<words.size();i++)
			{
				object_name+=words[i];

				if(i!=words.size()-1)
					object_name+=" ";
			}

			forget_object(object_name);
		}
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "orbit_demo");

	ros::NodeHandle private_nh_;

	client_talker = private_nh_.serviceClient<qbo_talk::Text2Speach>("/qbo_talk/festival_say_no_wait");
	client_self_recognizer = private_nh_.serviceClient<qbo_self_recognizer::QboRecognize>("qbo_self_recognizer/recognize");
	client_learn = private_nh_.serviceClient<qbo_object_recognition::LearnNewObject>("/qbo_object_recognition/learn");
	client_teach = private_nh_.serviceClient<qbo_object_recognition::Teach>("/qbo_object_recognition/teach");
	client_recognize = private_nh_.serviceClient<qbo_object_recognition::RecognizeObject>("/qbo_object_recognition/recognize_with_stabilizer");
	client_update= private_nh_.serviceClient<qbo_object_recognition::Update>("/qbo_object_recognition/update");

	//listener_sub =private_nh_.subscribe<qbo_listen::Listened>("/listen/en_object_recog",1,&listenerCallback);
	listener_sub =private_nh_.subscribe<qbo_listen::Listened>("/listen/en_default",1,&listenerCallback);

	//Subscribe to stereo selector images
        ros::Subscriber	image_sub=private_nh_.subscribe<sensor_msgs::Image>("/qbo_stereo_selector/object",1,&stereoSelectorCallback);
	
	ROS_INFO("Demo 2 Launched. Ready for incoming orders");

//	speak_this("I am ready to recognize objects");

	ros::spin();

  return 0;
}
