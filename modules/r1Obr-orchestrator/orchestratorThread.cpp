﻿/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "orchestratorThread.h"


YARP_LOG_COMPONENT(R1OBR_ORCHESTRATOR_THREAD, "r1_obr.orchestrator.orchestratorThread")


/****************************************************************/
OrchestratorThread::OrchestratorThread(yarp::os::ResourceFinder &rf):
    TypedReaderCallback(),
    m_rf(rf),
    m_status(R1_IDLE),
    m_where_specified(false),
    m_object_found(false),
    m_object_not_found(false),
    m_going(false)
{
    //Defaults
    m_sensor_network_rpc_port_name  = "/r1Obr-orchestrator/sensor_network:rpc";
    m_nextLoc_rpc_port_name         = "/r1Obr-orchestrator/nextLocPlanner/request:rpc";
    m_goandfindit_rpc_port_name     = "/r1Obr-orchestrator/goAndFindIt/request:rpc";
    m_goandfindit_result_port_name  = "/r1Obr-orchestrator/goAndFindIt/result:i";
    m_positive_outcome_port_name    = "/r1Obr-orchestrator/positive_outcome:o";
    m_negative_outcome_port_name    = "/r1Obr-orchestrator/negative_outcome:o";
    m_faceexpression_rpc_port_name  = "/r1Obr-orchestrator/faceExpression:rpc";
    m_map_prefix = "";
}

/****************************************************************/
bool OrchestratorThread::threadInit()
{
    // --------- ports config --------- //
    if (m_rf.check("sensor_network_rpc_port"))  {m_sensor_network_rpc_port_name   = m_rf.find("sensor_network_rpc_port").asString();}
    if (m_rf.check("next_loc_planner_rpc_port")){m_nextLoc_rpc_port_name          = m_rf.find("next_loc_planner_rpc_port").asString();}
    if (m_rf.check("goandfindit_rpc_port"))     {m_goandfindit_rpc_port_name      = m_rf.find("goandfindit_rpc_port").asString();}
    if (m_rf.check("goandfindit_result_port"))  {m_goandfindit_result_port_name   = m_rf.find("goandfindit_result_port").asString();}
    if (m_rf.check("faceexpression_rpc_port"))  {m_faceexpression_rpc_port_name   = m_rf.find("faceexpression_rpc_port").asString();}

    if(m_rf.check("map_prefix")){m_map_prefix = m_rf.find("map_prefix").asString();} 
    
    if(!m_sensor_network_rpc_port.open(m_sensor_network_rpc_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open Sensor Network RPC port with name" << m_sensor_network_rpc_port_name;
        return false;
    }

    if(!m_nextLoc_rpc_port.open(m_nextLoc_rpc_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open nextLocPlanner RPC port with name" << m_nextLoc_rpc_port_name;
        return false;
    }

    if(!m_goandfindit_rpc_port.open(m_goandfindit_rpc_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open goAndFindIt RPC port with name" << m_goandfindit_rpc_port_name;
        return false;
    }

    if(!m_goandfindit_result_port.open(m_goandfindit_result_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open goAndFindIt result port with name" << m_goandfindit_result_port_name;
        return false;
    }

    if(!m_faceexpression_rpc_port.open(m_faceexpression_rpc_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open faceExpression RPC port with name" << m_faceexpression_rpc_port_name;
        return false;
    }


    // --------- output ports config --------- //
    if(!m_rf.check("OUTPUT_PORT_GROUP"))
    {
        yCWarning(R1OBR_ORCHESTRATOR_THREAD,"OUTPUT_PORT_GROUP section missing in ini file. Using the default values");
    }
    else
    {
        Searchable& outputs = m_rf.findGroup("OUTPUT_PORT_GROUP");
        if(outputs.check("positive_outcome_port")) {m_positive_outcome_port_name = outputs.find("positive_outcome_port").asString();}
        if(outputs.check("negative_outcome_port")) {m_negative_outcome_port_name = outputs.find("negative_outcome_port").asString();}
    }

    if(!m_positive_outcome_port.open(m_positive_outcome_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open findObject RPC port with name" << m_positive_outcome_port_name;
        return false;
    }

    if(!m_negative_outcome_port.open(m_negative_outcome_port_name)){
        yCError(R1OBR_ORCHESTRATOR_THREAD) << "Cannot open nextOrient RPC port with name" << m_negative_outcome_port_name;
        return false;
    }

    // --------- Nav2Loc config --------- //
    m_nav2loc = new Nav2Loc();
    if(!m_nav2loc->configure(m_rf))
    {
        yCError(R1OBR_ORCHESTRATOR_THREAD,"Nav2Loc configuration failed");
        return false;
    }

    // --------- ContinousSearch config --------- //
    m_continuousSearch = new ContinuousSearch();
    if(!m_continuousSearch->configure(m_rf))
    {
        yCError(R1OBR_ORCHESTRATOR_THREAD,"ContinuousSearch configuration failed");
        return false;
    }

    // --------- Chat Bot initialization --------- //
    m_chat_bot = new ChatBot();
    if (!m_chat_bot->configure(m_rf)){
        return false;
    }
    string chat_bot_port_name = "/r1Obr-orchestrator/chatBot:rpc";
    string rpc_server_port_name = m_rf.check("input_rpc_port", Value("/r1Obr-orchestrator/rpc")).asString();
    bool ok = Network::connect(chat_bot_port_name.c_str(), rpc_server_port_name.c_str());
    if (!ok)
    {
        yCError(R1OBR_ORCHESTRATOR_THREAD,"Could not connect %s to %s", chat_bot_port_name.c_str(), rpc_server_port_name.c_str());
        return false;
    }

    return true;
}

/****************************************************************/
void OrchestratorThread::threadRelease()
{
    if (m_sensor_network_rpc_port.asPort().isOpen())
        m_sensor_network_rpc_port.close(); 
        
    if (m_nextLoc_rpc_port.asPort().isOpen())
        m_nextLoc_rpc_port.close(); 
        
    if (m_goandfindit_rpc_port.asPort().isOpen())
        m_goandfindit_rpc_port.close(); 
    
    if(!m_goandfindit_result_port.isClosed())
        m_goandfindit_result_port.close();

    if(!m_positive_outcome_port.isClosed())
        m_positive_outcome_port.close();

    if(!m_negative_outcome_port.isClosed())
        m_negative_outcome_port.close();
        
    if (m_faceexpression_rpc_port.asPort().isOpen())
        m_faceexpression_rpc_port.close(); 
    
    m_nav2loc->close();
    delete m_nav2loc;

    m_continuousSearch->close();
    delete m_continuousSearch;

    m_chat_bot->close();
    delete m_chat_bot;

    yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Orchestrator thread released");

    return;
}

/****************************************************************/
void OrchestratorThread::run()
{
    while (true)
    {
        setEmotion();
        
        if (m_status == R1_ASKING_NETWORK)
        {
            if (!askNetwork())
            {
                forwardRequest(m_request);
                m_status = R1_SEARCHING;
                yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Requested: %s", m_request.toString().c_str());
            }
        }

        else if (m_status == R1_SEARCHING)
        {
            Bottle req{"status"};
            string goandfindit_status = forwardRequest(req).get(0).asString();

            if(goandfindit_status == "navigating")
            {
                bool doContSearch = !m_where_specified || m_nav2loc->areYouNearToGoal();
                if(doContSearch && m_continuousSearch->seeObject(m_object))
                {
                    stopOrReset("stop");
                    m_status = R1_CONTINUOUS_SEARCH;
                    Time::delay(2.0);  //stopping the navigation the robot starts oscillating, better wait a couple of seconds before continuing
                    yCInfo(R1OBR_ORCHESTRATOR_THREAD, "I thought I saw a %s", m_object.c_str());
                    askChatBotToSpeak(object_found_maybe);
                }
            }
            else 
            {
                Bottle* result = m_goandfindit_result_port.read(false); 
                if(result != nullptr)
                {
                    m_result = *result;
                    if (result->get(0).asString() == "not found")  
                    {
                        if(m_where_specified)
                        {
                            yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Object not found in the specified location");
                        }
                        
                        m_status = R1_OBJECT_NOT_FOUND;
                    }
                    else     
                    {
                        m_status = R1_OBJECT_FOUND;
                        Bottle&  sendOk = m_positive_outcome_port.prepare();
                        sendOk.clear();
                        sendOk = m_result;
                        m_positive_outcome_port.write();
                        yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Object found");
                    }
                }
                else if( goandfindit_status == "idle") 
                {
                    m_status = R1_IDLE;
                    askChatBotToSpeak(something_bad_happened);
                }
            }
        }

        else if (m_status == R1_CONTINUOUS_SEARCH)
        {
            Bottle&  sendOk = m_positive_outcome_port.prepare();
            sendOk.clear();
            sendOk.addString(m_object);
            Bottle& obj_coords = sendOk.addList();
            if (m_continuousSearch->whereObject(m_object, obj_coords) && m_status == R1_CONTINUOUS_SEARCH) //second condition added in case of external stop 
            {
                m_status = R1_OBJECT_FOUND;
                m_positive_outcome_port.write();
                yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Object found while navigating");
            }
            else     
            {
                m_status = R1_SEARCHING;
                yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Object actually not found. Resuming navigation");
                askChatBotToSpeak(object_found_false);
                resume();
            }
        }

        else if (m_status == R1_OBJECT_FOUND)
        {
            m_object_found = true;
        }

        else if (m_status == R1_OBJECT_NOT_FOUND)
        {
            yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Object not found");
            m_object_not_found = true;

            m_nav2loc->goHome();
            bool arrived{false};
            while (!arrived && m_status == R1_OBJECT_NOT_FOUND )
            {
                arrived = m_nav2loc->areYouArrived();
                Time::delay(0.5);
            }

            if (m_status == R1_OBJECT_NOT_FOUND) //in case of external stop
            {
                askChatBotToSpeak(object_not_found);
                Bottle&  sendKo = m_negative_outcome_port.prepare();
                sendKo.clear();
                sendKo = m_result;
                m_negative_outcome_port.write();

                m_status = R1_IDLE;
            }
        } 

        else if (m_status == R1_GOING)
        {
            bool arrived{false};
            bool nav_aborted{false};
            while (!arrived && !nav_aborted && m_status == R1_GOING)
            {
                arrived = m_nav2loc->areYouArrived();
                nav_aborted = m_nav2loc->isNavigationAborted();
                Time::delay(0.2);
            }

            if (nav_aborted)
            {
                askChatBotToSpeak(go_target_not_reached);
                m_status = R1_IDLE;
            }

            
            if (m_status == R1_GOING) //in case of external stop
            {
                askChatBotToSpeak(go_target_reached);
                m_status = R1_IDLE;
            }
        }

        else if (m_status == R1_IDLE)
        {
            Time::delay(0.8);
        }

        else if (m_status == R1_STOP)
        {
            break;
        }

        Time::delay(0.2);
    }
}

/****************************************************************/
void OrchestratorThread::onRead(yarp::os::Bottle &b)
{
    yCInfo(R1OBR_ORCHESTRATOR_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 0)
    {
        yCError(R1OBR_ORCHESTRATOR_THREAD,"The input request bottle has the wrong number of elements");
    }
    else 
    {
        string cmd {b.get(0).asString()};

        if (cmd=="stop" || cmd=="reset") 
        { 
            stopOrReset(cmd);
        }
        if (cmd=="reset_home") 
        { 
            resetHome();
        }
        else if (cmd=="navpos") 
        {
            if(!setNavigationPosition())
            {
                yCWarning(R1OBR_ORCHESTRATOR_THREAD, "Cannot set navigation position now");
            } 
        }
        else if (cmd=="resume") 
        {
            resume();
        }
        else if (cmd=="search")
        {
            if (m_status == R1_OBJECT_FOUND || m_status == R1_OBJECT_NOT_FOUND )
                stopOrReset("stop");

            search(b);            
        }
        else if (cmd=="go")
        {
            go(b.get(1).asString());           
        }
        else
        {
            yCError(R1OBR_ORCHESTRATOR_THREAD, "Error: input command bottle not valid");
        }
    }
}

/****************************************************************/
void OrchestratorThread::onStop()
{
    Bottle request{"stop"};
    forwardRequest(request);
    m_status = R1_STOP;
}

/****************************************************************/
Bottle OrchestratorThread::forwardRequest(const Bottle& request)
{
    Bottle _rep_;
    m_goandfindit_rpc_port.write(request,_rep_);

    return _rep_;
}

/****************************************************************/
void OrchestratorThread::search(const Bottle& btl)
{
    if (m_status == R1_OBJECT_NOT_FOUND || m_status == R1_OBJECT_FOUND || m_status == R1_GOING)
    {
        stopOrReset("stop");
    } 

    m_object_found = false;
    m_object_not_found = false;
    m_going = false;

    if(resizeSearchBottle(btl))
    {
        m_status = R1_ASKING_NETWORK;
    }
    else
    {
        stopOrReset("stop"); 
    }  
        
}

/****************************************************************/
bool OrchestratorThread::resizeSearchBottle(const Bottle& btl)
{
    int sz = btl.size();

    if (sz > 2)
        m_where_specified = true;
    else 
        m_where_specified = false;

    m_request.clear();
    for (int i=0; i < min(sz,3); i++)
    {
        if(i==2)
        {
            string loc = btl.get(i).asString();
            
            if (loc.find(m_map_prefix) == string::npos) 
            {
                loc = m_map_prefix + loc;
            }
            
            Bottle req,rep;
            req.fromString("find " + loc);
            m_nextLoc_rpc_port.write(req,rep); //check if location name is valid
            if (rep.get(0).asString() != "ok")
            {
                yCError(R1OBR_ORCHESTRATOR_THREAD,"Location specified is not valid.");
                m_request.clear();
                askChatBotToSpeak(something_bad_happened);
                return false;
            }  
            m_request.addString(loc); 
        }
        else
            m_request.addString(btl.get(i).asString());
    }

    return true;
}

/****************************************************************/
bool OrchestratorThread::askNetwork()
{
    
    // NOT IMPLEMENTED YET

    return false;
}

/****************************************************************/
string OrchestratorThread::stopOrReset(const string& cmd)
{
    Bottle rep, request{cmd};
    forwardRequest(request); 
    
    if (m_status == R1_OBJECT_FOUND)
    {
        Bottle&  send = m_positive_outcome_port.prepare();
        send.clear();
        send.addString("stop");
        m_positive_outcome_port.write();
    } 
    else if (m_status == R1_OBJECT_NOT_FOUND || m_status == R1_GOING || m_nav2loc->areYouMoving())
    {
        m_nav2loc->stop();
    } 

    if (cmd == "reset")
    {
        m_object_found = false;
        m_object_not_found = false;
        m_going = false;
        setNavigationPosition();
    }

    m_status = R1_IDLE;
    return cmd + " executed";
}

/****************************************************************/
string OrchestratorThread::resetHome()
{
    stopOrReset("reset");

    m_status = R1_GOING;
    m_nav2loc->goHome();

    return "reset and sent home";
}


/****************************************************************/
string OrchestratorThread::resume()
{
    yCInfo(R1OBR_ORCHESTRATOR_THREAD, "Resuming");

    if (m_object_found)
    {
        if(!setNavigationPosition())
        {
            m_status = R1_IDLE;
            return "not resumed";
        }
        
        Bottle&  send = m_positive_outcome_port.prepare();
        send.clear();
        send.addString("resume");
        m_positive_outcome_port.write();
        
        m_status = R1_OBJECT_FOUND;

        return "resume: object found";
    }
    else if (m_object_not_found)
    {
        if(!setNavigationPosition())
        {
            m_status = R1_IDLE;
            return "not resumed";
        }

        m_status = R1_OBJECT_NOT_FOUND;
        return "resume: object not found";
    }
    else if (m_going)
    {
        m_nav2loc->resumeGo();

        m_status = R1_GOING;
        return "resume: go";
    }
    else
    {
        Bottle rep, request{"resume"};
        rep = forwardRequest(request); 

        request.clear();
        request.addString("status");
        if(forwardRequest(request).get(0).asString() != "idle") 
            m_status = R1_SEARCHING;
        
        return "search resumed";
    }

}

/****************************************************************/
void OrchestratorThread::objectFound()
{
    if (m_status == R1_OBJECT_FOUND) //in case meanwhile anything happened
    {
        askChatBotToSpeak(object_found_true);
        m_status = R1_IDLE;
    }
}

/****************************************************************/
void OrchestratorThread::objectActuallyNotFound() //in case we lose the sight of the object while approaching it
{
    if (m_status == R1_OBJECT_FOUND)
    {
        m_status = R1_SEARCHING;
        m_object_found = false;

        Bottle req;
        req.fromString("set " + getWhere() + " unchecked");
        m_nextLoc_rpc_port.write(req);

        resume();
    }
}

/****************************************************************/
void OrchestratorThread::setObject(string obj)
{
    m_object = obj;
}

/****************************************************************/
bool OrchestratorThread::setNavigationPosition()
{
    Bottle request{"navpos"};
    if(forwardRequest(request).get(0).asString() != "setting robot in navigation position")
    {
        askChatBotToSpeak(hardware_failure);
        return false;
    }
    
    return true;
}


/****************************************************************/
string OrchestratorThread::getWhat()
{
    Bottle request{"what"};
    return forwardRequest(request).toString();
}


/****************************************************************/
string OrchestratorThread::getWhere()
{
    if(m_status == R1_GOING)
        return m_nav2loc->getCurrentTarget();
    
    Bottle request{"where"};
    return forwardRequest(request).toString();
}


/****************************************************************/
string OrchestratorThread::getStatus()
{
    Bottle request{"status"}, reply;
    m_goandfindit_rpc_port.write(request,reply);
    string goAndFindItStatus = reply.get(0).asString();
    string str;

    switch (m_status)
    {
    case R1_IDLE:
        str = "idle";
        break;
    case R1_ASKING_NETWORK:
        str = "asking_network";
        break;
    case R1_SEARCHING:
        str = "searching - " + goAndFindItStatus;
        break;
    case R1_CONTINUOUS_SEARCH:
        str = "object_found_while_navigating";
        break;
    case R1_OBJECT_FOUND:
        str = "object_found";
        break;
    case R1_OBJECT_NOT_FOUND:
        str = "object_not_found";
        break;
    case R1_GOING:
        str = "going";
        break;
    case R1_STOP:
        str = "stop";
        break;
    default:
        str = "ERROR";
        break;
    };

    return str;
}


/****************************************************************/
void OrchestratorThread::info(Bottle& reply)
{
    reply.clear();
    reply.addVocab32("many");

    Bottle& whatList=reply.addList();
    whatList.addString("what"); whatList.addString(":");
    whatList.addString(getWhat());

    Bottle& whereList=reply.addList();
    whereList.addString("where"); whereList.addString(":");
    whereList.addString(getWhere());

    Bottle& statusList=reply.addList();
    statusList.addString("status"); statusList.addString(":");
    statusList.addString(getStatus());
}


/****************************************************************/
void OrchestratorThread::setEmotion()
{
    Bottle request, reply;
    if (m_status == R1_OBJECT_FOUND || m_object_found)
        request.fromString("emotion 1"); //happy
    else if (m_status == R1_OBJECT_NOT_FOUND || m_object_not_found)
        request.fromString("emotion 0"); //sad
    else if (getStatus() == "searching - searching" || m_status == R1_ASKING_NETWORK)
        request.fromString("emotion 2"); //thinking
    else 
        request.fromString("emotion 1"); //happy
    
    m_faceexpression_rpc_port.write(request,reply);
}


/****************************************************************/
bool OrchestratorThread::askChatBotToSpeak(R1_says stat)
{
    string str;
    switch (stat)
    {
    case object_found_maybe:
        str = "object_found_maybe";
        break;
    case object_found_true:
        str = "object_found_true";
        break;
    case object_found_false:
        str = "object_found_false";
        break;
    case object_not_found:
        str = "object_not_found";
        break;
    case something_bad_happened:
        str = "something_bad_happened";
        break;
    case go_target_reached:
        str = "go_target_reached";
        break;
    case go_target_not_reached:
        str = "destination_not_reached";
        break;
    case hardware_failure:
        str = "hardware_failure";
        break;
    default:
        str = "fallback";
        break;
    };

    m_chat_bot->interactWithChatBot(str);    
    
    return true;
}


/****************************************************************/
bool OrchestratorThread::go(string loc)
{
    if (m_status != R1_IDLE) 
        stopOrReset("stop");
    
    if (loc != "home" && loc.find(m_map_prefix) == string::npos) 
    {
        loc = m_map_prefix + loc;
    }
    
    m_object_found = false;
    m_object_not_found = false;
    
    if (loc != "home")
    {
        Bottle req,rep;
        req.fromString("find " + loc);
        m_nextLoc_rpc_port.write(req,rep); //check if location name is valid
        if (rep.get(0).asString() != "ok")
        {
            yCError(R1OBR_ORCHESTRATOR_THREAD,"Location specified is not valid.");
            askChatBotToSpeak(something_bad_happened);
            return false;
        }
    }

    if(!setNavigationPosition())
    {
        m_status = R1_IDLE;
        return false;
    }

    m_status = R1_GOING;
    m_going = true;
    return m_nav2loc->go(loc);
}
