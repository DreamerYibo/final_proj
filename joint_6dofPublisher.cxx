// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file joint_6dofPublisher.cpp
 * This file contains the implementation of the publisher functions.
 *
 * This file was generated by the tool fastcdrgen.
 */

#include "joint_6dofPublisher.h"
#include "joint_6dofPubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <thread>
#include <chrono>

using namespace eprosima::fastdds::dds;

joint_6dofPublisher::joint_6dofPublisher()
    : participant_(nullptr), publisher_(nullptr), topic_(nullptr), writer_(nullptr), type_(new joint_6dofPubSubType())
{
}

joint_6dofPublisher::~joint_6dofPublisher()
{
    if (writer_ != nullptr)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

bool joint_6dofPublisher::init()
{
    /* Initialize data_ here */

    //CREATE THE PARTICIPANT
    DomainParticipantQos pqos;
    pqos.name("Participant_pub");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, pqos);
    if (participant_ == nullptr)
    {
        return false;
    }

    //REGISTER THE TYPE
    type_.register_type(participant_);

    //CREATE THE PUBLISHER
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
    if (publisher_ == nullptr)
    {
        return false;
    }

    //CREATE THE TOPIC
    topic_ = participant_->create_topic(
        "joint_6dofTopic",
        type_.get_type_name(),
        TOPIC_QOS_DEFAULT);
    if (topic_ == nullptr)
    {
        return false;
    }

    // CREATE THE WRITER
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);
    if (writer_ == nullptr)
    {
        return false;
    }

    std::cout << "joint_6dof DataWriter created." << std::endl;
    return true;
}

void joint_6dofPublisher::PubListener::on_publication_matched(
    eprosima::fastdds::dds::DataWriter *,
    const eprosima::fastdds::dds::PublicationMatchedStatus &info)
{
    if (info.current_count_change == 1)
    {
        matched = info.total_count;
        std::cout << "DataWriter matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched = info.total_count;
        std::cout << "DataWriter unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    }
}

void joint_6dofPublisher::run()
{
    std::cout << "joint_6dof DataWriter waiting for DataReaders." << std::endl;
    while (listener_.matched == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Sleep 250 ms
    }

    // Publication code

    joint_6dof st;
    st.joint_name() = "robot1";
    st.joint_pos()[0] = 0;
    st.joint_pos()[1] = 0;
    st.joint_pos()[2] = 0;
    st.joint_pos()[3] = 0;
    st.joint_pos()[4] = 0;
    st.joint_pos()[5] = 0;
    /* Initialize your structure here */

    int msgsent = 0;

    for (int i = 0; i < 500; i++)
    {
        writer_->write(&st);
        ++msgsent;
        std::cout << "Sending sample, count=" << msgsent << "\n";
        for (int j = 0; j < 6; j++)
        {
            st.joint_pos()[j] += 0.01;
        }
        st.joint_name() = "robot" + std::to_string(i % 3 + 1);
        //for debug //std::cout << "sent robot name: " << st.joint_name() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
}

void joint_6dofPublisher::publish(Eigen::MatrixXd &joint_pos_mat, int index)
{
    while (listener_.matched == 0)
    {
        std::cout << "Waiting for subscriber\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Sleep 250 ms
    }
    static joint_6dof st;

    // switch (index)
    // {
    // case 1:
    //     st.joint_name() = "robot1";
    //     break;
    // case 2:
    //     st.joint_name() = "robot2";
    //     break;
    // case 3:
    //     st.joint_name() = "robot3";
    //     break;
    // default:
    //     std::cout << "publisher: Invalid index number! Must be 1, 2 or 3!\n";
    //     break;
    // }

    if(index == 1 || index == 2 || index == 3)
    {
        st.joint_name() = "robot" + std::to_string(index);
    }
    else
    {
        std::cout << "publisher: Invalid index number! Must be 1, 2 or 3!\n";
    }
    for(int i=0; i<6; i++)
    {
        st.joint_pos()[i] = joint_pos_mat(i);
    }
    writer_->write(&st);
    //std::cout << "sent robot name: " << st.joint_name() << "\n";
    return ;
}