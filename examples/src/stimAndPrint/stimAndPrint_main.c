/*
* (c) Copyright, Real-Time Innovations, 2022.  All rights reserved.
* RTI grants Licensee a license to use, modify, compile, and create derivative
* works of the software solely for use with RTI Connext DDS. Licensee may
* redistribute copies of the software provided that all such copies are subject
* to this license. The software is provided "as is", with no warranty of any
* type, including any warranty for fitness for any purpose. RTI is under no
* obligation to maintain or support the software. RTI shall not be liable for
* any incidental or consequential damages arising out of the use or inability
* to use the software.
*/
/* stimAndPrint_main.c
* An application skeleton using the following DDS data types:
*   dds::sensing::Lidar
*   dds::sensing::Camera
*   dds::sensing::imu::Imu_real
*   dds::sensing::Compass
*   dds::sensing::gnss::Basic
*   dds::planning::MapData
*   dds::system::StatusTopic
* This skeleton file is generated, but intended to be used
* as a starting point example and modified by the user
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rti_me_c.h"
#include "wh_sm/wh_sm_history.h"
#include "rh_sm/rh_sm_history.h"
/* include the type support headers for the needed data types */
#include "res/types/data/sensing/Lidar_tSupport.h"
#include "res/types/data/sensing/Camera_tSupport.h"
#include "res/types/data/sensing/Imu_tSupport.h"
#include "res/types/data/sensing/Compass_tSupport.h"
#include "res/types/data/sensing/Gnss_tSupport.h"
#include "res/types/data/planning/Maps_tSupport.h"
#include "res/types/data/system/StatusTopic_tSupport.h"
#include "stimAndPrint_dp.h"

/* DATA WRITER LISTENERS: called when pub is matched to a subscriber */
void
dds_sensing_LidarPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

void
dds_sensing_CameraPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

void
dds_sensing_imu_Imu_realPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

void
dds_sensing_CompassPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

void
dds_sensing_gnss_BasicPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

void
dds_planning_MapDataPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a subscriber\n");
    }
}

/* DATA READER LISTENERS: callbacks activated when:
   - sub is matched to a publisher,
   - data sample has been received.
*/
void
dds_system_StatusTopicSubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a publisher\n");
    }
}

void
dds_system_StatusTopicSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_system_StatusTopicDataReader *hw_reader = dds_system_StatusTopicDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_system_StatusTopic *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_system_StatusTopicSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_system_StatusTopicDataReader_take(
        hw_reader,
        &sample_seq,
        &info_seq,
        DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE,
        DDS_ANY_VIEW_STATE,
        DDS_ANY_INSTANCE_STATE);

    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to take data, retcode(%d)\n", retcode);
        goto done;
    }

    /* Print each valid sample taken */
    for (i = 0; i < dds_system_StatusTopicSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_system_StatusTopicSeq_get_reference(&sample_seq, i);
            /* printf("\nValid sample received\n"); */
            *total_samples += 1;

            /* TODO read and process sample attributes here */
            printf("Status: %s\n", sample->data);

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_system_StatusTopicDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_system_StatusTopicSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}


/* main loop of the application ---------------------------- */
int
application_main_w_args(
    DDS_Long domain_id,
    char *udp_intf,
    char *peer,
    DDS_Long sleep_time,
    DDS_Long count)
{
    struct Application *application = NULL;
    DDS_Publisher *publisher;
    DDS_DataWriter *Lidar_datawriter;
    dds_sensing_LidarDataWriter *Lidar_hw_datawriter;
    struct DDS_DataWriterQos Lidar_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_sensing_Lidar *Lidar_sample = NULL;

    DDS_DataWriter *Camera_datawriter;
    dds_sensing_CameraDataWriter *Camera_hw_datawriter;
    struct DDS_DataWriterQos Camera_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_sensing_Camera *Camera_sample = NULL;

    DDS_DataWriter *Imu_real_datawriter;
    dds_sensing_imu_Imu_realDataWriter *Imu_real_hw_datawriter;
    struct DDS_DataWriterQos Imu_real_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_sensing_imu_Imu_real *Imu_real_sample = NULL;

    DDS_DataWriter *Compass_datawriter;
    dds_sensing_CompassDataWriter *Compass_hw_datawriter;
    struct DDS_DataWriterQos Compass_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_sensing_Compass *Compass_sample = NULL;

    DDS_DataWriter *Basic_datawriter;
    dds_sensing_gnss_BasicDataWriter *Basic_hw_datawriter;
    struct DDS_DataWriterQos Basic_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_sensing_gnss_Basic *Basic_sample = NULL;

    DDS_DataWriter *MapData_datawriter;
    dds_planning_MapDataDataWriter *MapData_hw_datawriter;
    struct DDS_DataWriterQos MapData_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_planning_MapData *MapData_sample = NULL;

    DDS_Subscriber *subscriber;
    DDS_DataReader *StatusTopic_datareader;
    struct DDS_DataReaderQos StatusTopic_dr_qos = DDS_DataReaderQos_INITIALIZER;

    struct DDS_DataWriterListener Lidar_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Lidar_sample = dds_sensing_LidarTypeSupport_create_data();
    if (Lidar_sample == NULL)
    {
        printf("failed dds_sensing_LidarTypeSupport_create_data\n");
        return -1;
    }

    struct DDS_DataWriterListener Camera_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Camera_sample = dds_sensing_CameraTypeSupport_create_data();
    if (Camera_sample == NULL)
    {
        printf("failed dds_sensing_CameraTypeSupport_create_data\n");
        return -1;
    }

    struct DDS_DataWriterListener Imu_real_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Imu_real_sample = dds_sensing_imu_Imu_realTypeSupport_create_data();
    if (Imu_real_sample == NULL)
    {
        printf("failed dds_sensing_imu_Imu_realTypeSupport_create_data\n");
        return -1;
    }

    struct DDS_DataWriterListener Compass_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Compass_sample = dds_sensing_CompassTypeSupport_create_data();
    if (Compass_sample == NULL)
    {
        printf("failed dds_sensing_CompassTypeSupport_create_data\n");
        return -1;
    }

    struct DDS_DataWriterListener Basic_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Basic_sample = dds_sensing_gnss_BasicTypeSupport_create_data();
    if (Basic_sample == NULL)
    {
        printf("failed dds_sensing_gnss_BasicTypeSupport_create_data\n");
        return -1;
    }

    struct DDS_DataWriterListener MapData_dw_listener = DDS_DataWriterListener_INITIALIZER;
    MapData_sample = dds_planning_MapDataTypeSupport_create_data();
    if (MapData_sample == NULL)
    {
        printf("failed dds_planning_MapDataTypeSupport_create_data\n");
        return -1;
    }
    struct DDS_DataReaderListener StatusTopic_dr_listener = DDS_DataReaderListener_INITIALIZER;
    DDS_ReturnCode_t retcode;

    int ret_value = -1;
    DDS_Long total_samples = 0;
    DDS_Long i = 0;

    /* create and init the DDS domain participant */
    application = Application_create(domain_id, udp_intf, peer);
    if (application == NULL)
    {
        printf("domain participant creation error\n");
        goto done;
    }
    application->sleep_time = sleep_time;
    application->count = count;

    /* register & create topics for each pub|sub */
    char type_name_tmp[255];
    char topic_name_tmp[255];
    /* Lidar topic */
    DDS_Topic *Lidar_topic;
    strcpy(type_name_tmp, dds_sensing_LidarTypeSupport_get_type_name());
    retcode = dds_sensing_LidarTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "lidarPoints");
    Lidar_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Lidar_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* Camera topic */
    DDS_Topic *Camera_topic;
    strcpy(type_name_tmp, dds_sensing_CameraTypeSupport_get_type_name());
    retcode = dds_sensing_CameraTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "cameraView");
    Camera_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Camera_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* Imu_real topic */
    DDS_Topic *Imu_real_topic;
    strcpy(type_name_tmp, dds_sensing_imu_Imu_realTypeSupport_get_type_name());
    retcode = dds_sensing_imu_Imu_realTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "imuData");
    Imu_real_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Imu_real_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* Compass topic */
    DDS_Topic *Compass_topic;
    strcpy(type_name_tmp, dds_sensing_CompassTypeSupport_get_type_name());
    retcode = dds_sensing_CompassTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "compass");
    Compass_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Compass_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* Basic topic */
    DDS_Topic *Basic_topic;
    strcpy(type_name_tmp, dds_sensing_gnss_BasicTypeSupport_get_type_name());
    retcode = dds_sensing_gnss_BasicTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "gnssPosition");
    Basic_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Basic_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* MapData topic */
    DDS_Topic *MapData_topic;
    strcpy(type_name_tmp, dds_planning_MapDataTypeSupport_get_type_name());
    retcode = dds_planning_MapDataTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "mapData");
    MapData_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (MapData_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* StatusTopic topic */
    DDS_Topic *StatusTopic_topic;
    strcpy(type_name_tmp, dds_system_StatusTopicTypeSupport_get_type_name());
    retcode = dds_system_StatusTopicTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "egoStatus");
    StatusTopic_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (StatusTopic_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* create a publisher ---------------------- */
    publisher = DDS_DomainParticipant_create_publisher(
        application->participant,
        &DDS_PUBLISHER_QOS_DEFAULT,
        NULL,
        DDS_STATUS_MASK_NONE);
    if (publisher == NULL)
    {
        printf("publisher == NULL\n");
        goto done;
    }

    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    Lidar_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Lidar_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Lidar_dw_qos.resource_limits.max_samples_per_instance = 32;
    Lidar_dw_qos.resource_limits.max_instances = 2;
    Lidar_dw_qos.resource_limits.max_samples = Lidar_dw_qos.resource_limits.max_instances *
        Lidar_dw_qos.resource_limits.max_samples_per_instance;
    Lidar_dw_qos.history.depth = 32;
    Lidar_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Lidar_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Lidar_dw_listener.on_publication_matched = dds_sensing_LidarPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Lidar_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Lidar_topic,
        &Lidar_dw_qos,
        &Lidar_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Lidar_datawriter == NULL)
    {
        printf("Lidar_datawriter == NULL\n");
        goto done;
    }

    Lidar_hw_datawriter = dds_sensing_LidarDataWriter_narrow(Lidar_datawriter);
    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    Camera_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Camera_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Camera_dw_qos.resource_limits.max_samples_per_instance = 32;
    Camera_dw_qos.resource_limits.max_instances = 2;
    Camera_dw_qos.resource_limits.max_samples = Camera_dw_qos.resource_limits.max_instances *
        Camera_dw_qos.resource_limits.max_samples_per_instance;
    Camera_dw_qos.history.depth = 32;
    Camera_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Camera_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Camera_dw_listener.on_publication_matched = dds_sensing_CameraPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Camera_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Camera_topic,
        &Camera_dw_qos,
        &Camera_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Camera_datawriter == NULL)
    {
        printf("Camera_datawriter == NULL\n");
        goto done;
    }

    Camera_hw_datawriter = dds_sensing_CameraDataWriter_narrow(Camera_datawriter);
    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    Imu_real_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Imu_real_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Imu_real_dw_qos.resource_limits.max_samples_per_instance = 32;
    Imu_real_dw_qos.resource_limits.max_instances = 2;
    Imu_real_dw_qos.resource_limits.max_samples = Imu_real_dw_qos.resource_limits.max_instances *
        Imu_real_dw_qos.resource_limits.max_samples_per_instance;
    Imu_real_dw_qos.history.depth = 32;
    Imu_real_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Imu_real_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Imu_real_dw_listener.on_publication_matched = dds_sensing_imu_Imu_realPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Imu_real_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Imu_real_topic,
        &Imu_real_dw_qos,
        &Imu_real_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Imu_real_datawriter == NULL)
    {
        printf("Imu_real_datawriter == NULL\n");
        goto done;
    }

    Imu_real_hw_datawriter = dds_sensing_imu_Imu_realDataWriter_narrow(Imu_real_datawriter);
    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    Compass_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Compass_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Compass_dw_qos.resource_limits.max_samples_per_instance = 32;
    Compass_dw_qos.resource_limits.max_instances = 2;
    Compass_dw_qos.resource_limits.max_samples = Compass_dw_qos.resource_limits.max_instances *
        Compass_dw_qos.resource_limits.max_samples_per_instance;
    Compass_dw_qos.history.depth = 32;
    Compass_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Compass_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Compass_dw_listener.on_publication_matched = dds_sensing_CompassPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Compass_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Compass_topic,
        &Compass_dw_qos,
        &Compass_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Compass_datawriter == NULL)
    {
        printf("Compass_datawriter == NULL\n");
        goto done;
    }

    Compass_hw_datawriter = dds_sensing_CompassDataWriter_narrow(Compass_datawriter);
    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    Basic_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Basic_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Basic_dw_qos.resource_limits.max_samples_per_instance = 32;
    Basic_dw_qos.resource_limits.max_instances = 2;
    Basic_dw_qos.resource_limits.max_samples = Basic_dw_qos.resource_limits.max_instances *
        Basic_dw_qos.resource_limits.max_samples_per_instance;
    Basic_dw_qos.history.depth = 32;
    Basic_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Basic_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Basic_dw_listener.on_publication_matched = dds_sensing_gnss_BasicPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Basic_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Basic_topic,
        &Basic_dw_qos,
        &Basic_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Basic_datawriter == NULL)
    {
        printf("Basic_datawriter == NULL\n");
        goto done;
    }

    Basic_hw_datawriter = dds_sensing_gnss_BasicDataWriter_narrow(Basic_datawriter);
    /* Reliability QoS */
#ifdef USE_RELIABLE_QOS
    MapData_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    MapData_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    MapData_dw_qos.resource_limits.max_samples_per_instance = 32;
    MapData_dw_qos.resource_limits.max_instances = 2;
    MapData_dw_qos.resource_limits.max_samples = MapData_dw_qos.resource_limits.max_instances *
        MapData_dw_qos.resource_limits.max_samples_per_instance;
    MapData_dw_qos.history.depth = 32;
    MapData_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    MapData_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    MapData_dw_listener.on_publication_matched = dds_planning_MapDataPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    MapData_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        MapData_topic,
        &MapData_dw_qos,
        &MapData_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (MapData_datawriter == NULL)
    {
        printf("MapData_datawriter == NULL\n");
        goto done;
    }

    MapData_hw_datawriter = dds_planning_MapDataDataWriter_narrow(MapData_datawriter);
    /* create a subscriber ---------------------- */
    subscriber = DDS_DomainParticipant_create_subscriber(
        application->participant,
        &DDS_SUBSCRIBER_QOS_DEFAULT,
        NULL,
        DDS_STATUS_MASK_NONE);
    if (subscriber == NULL)
    {
        printf("subscriber == NULL\n");
        goto done;
    }

    /* Publisher sends samples with id = 0 or id = 1, so 2 instances maximum.
    * But in case filtering is done, all samples with 'id = 0' are
    * filtered so only one instance is needed.
    */
    StatusTopic_dr_qos.resource_limits.max_instances = 2;
    StatusTopic_dr_qos.resource_limits.max_samples_per_instance = 32;
    StatusTopic_dr_qos.resource_limits.max_samples = StatusTopic_dr_qos.resource_limits.max_instances *
        StatusTopic_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    StatusTopic_dr_qos.reader_resource_limits.max_remote_writers = 10;
    StatusTopic_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    StatusTopic_dr_qos.history.depth = 32;

    StatusTopic_dr_listener.on_data_available = dds_system_StatusTopicSubscriber_on_data_available;
    StatusTopic_dr_listener.on_subscription_matched =
        dds_system_StatusTopicSubscriber_on_subscription_matched;
    StatusTopic_dr_listener.as_listener.listener_data = &total_samples;
    StatusTopic_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(StatusTopic_topic),
        &StatusTopic_dr_qos,
        &StatusTopic_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (StatusTopic_datareader == NULL)
    {
        printf("StatusTopic_datareader == NULL\n");
        goto done;
    }

#ifdef RTI_CERT
#ifdef RTI_VXWORKS
    /** End initialization, disable further dynamic memory allocation ***/
    memAllocDisable();
#endif  /* RTI_VXWORKS */
#endif  /* RTI_CERT */

    /* Initialize values in the pub topics */
    Lidar_sample->parent.id[0] = 1234;
    Lidar_sample->pointcloud.scale.x = 1.0;
    Lidar_sample->pointcloud.scale.y = 1.0;
    Lidar_sample->pointcloud.scale.z = 1.0;
    Camera_sample->parent.id[0] = 2345;
    Camera_sample->image.height = dds_sensing_image_MAX_HEIGHT;
    Camera_sample->image.width = dds_sensing_image_MAX_WIDTH;
    Camera_sample->image.format = RGB8;
    Imu_real_sample->parent.id[0] = 3456;
    Imu_real_sample->orientation.x = 0.2222;
    Imu_real_sample->orientation.y = -0.7777;
    Imu_real_sample->orientation.z = 0.0123;
    Compass_sample->parent.id[0] = 4567;
    Compass_sample->heading = 0.4444;
    Basic_sample->parent.id[0] = 5678;
    Basic_sample->lat = 36.286047729082135;
    Basic_sample->lon = -116.82601684339734;
    Basic_sample->alt = -84.5;
    MapData_sample->parent.id[0] = 6789;
    MapData_sample->seq_id = 0;

    /* loop ---------------- */
    for (i = 0; (application->count <= 0) || (i < application->count); ++i)
    {
        /* TODO set *_sample values/attributes here */
        Lidar_sample->pointcloud.point_return_count[0] = i;
        Imu_real_sample->angular_velocity.x = 1 + (i / 500);
        Compass_sample->heading = 0.5 + (i / 500);
        Basic_sample->alt = -84.5 + (i/500);
        MapData_sample->seq_id = i;

        retcode = dds_sensing_LidarDataWriter_write(
            Lidar_hw_datawriter,
            Lidar_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Lidar");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        retcode = dds_sensing_CameraDataWriter_write(
            Camera_hw_datawriter,
            Camera_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Camera");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        retcode = dds_sensing_imu_Imu_realDataWriter_write(
            Imu_real_hw_datawriter,
            Imu_real_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Imu_real");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        retcode = dds_sensing_CompassDataWriter_write(
            Compass_hw_datawriter,
            Compass_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Compass");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        retcode = dds_sensing_gnss_BasicDataWriter_write(
            Basic_hw_datawriter,
            Basic_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Basic");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        retcode = dds_planning_MapDataDataWriter_write(
            MapData_hw_datawriter,
            MapData_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "MapData");
        }
        else
        {
            printf("Written sample %d\n",(i+1));
        }

        printf("Subscriber sleeping for %d msec...\n", sleep_time);

        OSAPI_Thread_sleep(application->sleep_time);
    }

    /* Finished; clean up and exit */
    ret_value = 0;
done:

#ifndef RTI_CERT
    if (application != NULL)
    {
        Application_delete(application);
    }

    if (Lidar_sample != NULL)
    {
        dds_sensing_LidarTypeSupport_delete_data(Lidar_sample);
    }
    if (Camera_sample != NULL)
    {
        dds_sensing_CameraTypeSupport_delete_data(Camera_sample);
    }
    if (Imu_real_sample != NULL)
    {
        dds_sensing_imu_Imu_realTypeSupport_delete_data(Imu_real_sample);
    }
    if (Compass_sample != NULL)
    {
        dds_sensing_CompassTypeSupport_delete_data(Compass_sample);
    }
    if (Basic_sample != NULL)
    {
        dds_sensing_gnss_BasicTypeSupport_delete_data(Basic_sample);
    }
    if (MapData_sample != NULL)
    {
        dds_planning_MapDataTypeSupport_delete_data(MapData_sample);
    }

#endif  /* RTI_CERT */
#ifndef RTI_CERT
    retcode = DDS_DataWriterQos_finalize(&Lidar_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataWriterQos_finalize(&Camera_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataWriterQos_finalize(&Imu_real_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataWriterQos_finalize(&Compass_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataWriterQos_finalize(&Basic_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataWriterQos_finalize(&MapData_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&StatusTopic_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
#endif  /* RTI_CERT */
    if (ret_value == 0)
    {
        printf("Samples received %d\n", total_samples);
        if (total_samples == 0)
        {
            return -1;
        }
    }

    return ret_value;
}


#if !(defined(RTI_VXWORKS) && !defined(__RTP__))
int
main(int argc, char **argv)
{
    DDS_Long i = 0;
    DDS_Long domain_id = 0;
    char *peer = NULL;
    char *udp_intf = NULL;
    DDS_Long sleep_time = 1000;
    DDS_Long count = 0;

    for (i = 1; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-domain"))
        {
            ++i;
            if (i == argc)
            {
                printf("-domain <domain_id>\n");
                return -1;
            }
            domain_id = strtol(argv[i], NULL, 0);
        }
        else if (!strcmp(argv[i], "-udp_intf"))
        {
            ++i;
            if (i == argc)
            {
                printf("-udp_intf <interface>\n");
                return -1;
            }
            udp_intf = argv[i];
        }
        else if (!strcmp(argv[i], "-peer"))
        {
            ++i;
            if (i == argc)
            {
                printf("-peer <address>\n");
                return -1;
            }
            peer = argv[i];
        }
        else if (!strcmp(argv[i], "-sleep"))
        {
            ++i;
            if (i == argc)
            {
                printf("-sleep_time <sleep_time>\n");
                return -1;
            }
            sleep_time = strtol(argv[i], NULL, 0);
        }
        else if (!strcmp(argv[i], "-count"))
        {
            ++i;
            if (i == argc)
            {
                printf("-count <count>\n");
                return -1;
            }
            count = strtol(argv[i], NULL, 0);
        }
        else if (!strcmp(argv[i], "-h"))
        {
            Application_help(argv[0]);
            return 0;
        }
        else
        {
            printf("unknown option: %s\n", argv[i]);
            return -1;
        }
    }

    return application_main_w_args(domain_id, udp_intf, peer, sleep_time, count);
}

#elif defined(RTI_VXWORKS)
int
application_main(void)
{
    /* Explicitly configure args below */
    DDS_Long domain_id = 0;
    char *peer = "127.0.0.1";
    char *udp_intf = NULL;
    DDS_Long sleep_time = 1000;
    DDS_Long count = 0;

    return application_main_w_args(domain_id, udp_intf, peer, sleep_time, count);
}
#endif  /* defined(RTI_VXWORKS) */
