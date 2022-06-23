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
/* perception_main.c
* An application skeleton using the following DDS data types:
*   dds::sensing::Lidar
*   dds::sensing::Camera
*   dds::physics::Pose3D_real
*   dds::perception::Trajectory
*   dds::planning::MapData
*   dds::perception::ObjectsDetected
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
#include "res/types/data/physics/Pose3D_tSupport.h"
#include "res/types/data/perception/Trajectory_tSupport.h"
#include "res/types/data/planning/Maps_tSupport.h"
#include "res/types/data/perception/Object_tSupport.h"
#include "perception_dp.h"

/* DATA WRITER LISTENERS: called when pub is matched to a subscriber */
void
dds_perception_ObjectsDetectedPublisher_on_publication_matched(
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
dds_sensing_LidarSubscriber_on_subscription_matched(
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
dds_sensing_LidarSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_sensing_LidarDataReader *hw_reader = dds_sensing_LidarDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_sensing_Lidar *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_sensing_LidarSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_sensing_LidarDataReader_take(
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
    for (i = 0; i < dds_sensing_LidarSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_sensing_LidarSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_sensing_LidarDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_sensing_LidarSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_sensing_CameraSubscriber_on_subscription_matched(
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
dds_sensing_CameraSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_sensing_CameraDataReader *hw_reader = dds_sensing_CameraDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_sensing_Camera *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_sensing_CameraSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_sensing_CameraDataReader_take(
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
    for (i = 0; i < dds_sensing_CameraSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_sensing_CameraSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_sensing_CameraDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_sensing_CameraSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_physics_Pose3D_realSubscriber_on_subscription_matched(
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
dds_physics_Pose3D_realSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_physics_Pose3D_realDataReader *hw_reader = dds_physics_Pose3D_realDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_physics_Pose3D_real *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_physics_Pose3D_realSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_physics_Pose3D_realDataReader_take(
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
    for (i = 0; i < dds_physics_Pose3D_realSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_physics_Pose3D_realSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_physics_Pose3D_realDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_physics_Pose3D_realSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_perception_TrajectorySubscriber_on_subscription_matched(
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
dds_perception_TrajectorySubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_perception_TrajectoryDataReader *hw_reader = dds_perception_TrajectoryDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_perception_Trajectory *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_perception_TrajectorySeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_perception_TrajectoryDataReader_take(
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
    for (i = 0; i < dds_perception_TrajectorySeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_perception_TrajectorySeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_perception_TrajectoryDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_perception_TrajectorySeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_planning_MapDataSubscriber_on_subscription_matched(
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
dds_planning_MapDataSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_planning_MapDataDataReader *hw_reader = dds_planning_MapDataDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_planning_MapData *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_planning_MapDataSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_planning_MapDataDataReader_take(
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
    for (i = 0; i < dds_planning_MapDataSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_planning_MapDataSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_planning_MapDataDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_planning_MapDataSeq_finalize(&sample_seq);
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
    DDS_DataWriter *ObjectsDetected_datawriter;
    dds_perception_ObjectsDetectedDataWriter *ObjectsDetected_hw_datawriter;
    struct DDS_DataWriterQos ObjectsDetected_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_perception_ObjectsDetected *ObjectsDetected_sample = NULL;

    DDS_Subscriber *subscriber;
    DDS_DataReader *Lidar_datareader;
    struct DDS_DataReaderQos Lidar_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Camera_datareader;
    struct DDS_DataReaderQos Camera_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Pose3D_real_datareader;
    struct DDS_DataReaderQos Pose3D_real_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Trajectory_datareader;
    struct DDS_DataReaderQos Trajectory_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *MapData_datareader;
    struct DDS_DataReaderQos MapData_dr_qos = DDS_DataReaderQos_INITIALIZER;

    struct DDS_DataWriterListener ObjectsDetected_dw_listener = DDS_DataWriterListener_INITIALIZER;
    ObjectsDetected_sample = dds_perception_ObjectsDetectedTypeSupport_create_data();
    if (ObjectsDetected_sample == NULL)
    {
        printf("failed dds_perception_ObjectsDetectedTypeSupport_create_data\n");
        return -1;
    }
    struct DDS_DataReaderListener Lidar_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Camera_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Pose3D_real_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Trajectory_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener MapData_dr_listener = DDS_DataReaderListener_INITIALIZER;
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

    /* Pose3D_real topic */
    DDS_Topic *Pose3D_real_topic;
    strcpy(type_name_tmp, dds_physics_Pose3D_realTypeSupport_get_type_name());
    retcode = dds_physics_Pose3D_realTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "egoPose");
    Pose3D_real_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Pose3D_real_topic == NULL)
    {
        printf("topic %s == NULL\n", type_name_tmp);
        goto done;
    }

    /* Trajectory topic */
    DDS_Topic *Trajectory_topic;
    strcpy(type_name_tmp, dds_perception_TrajectoryTypeSupport_get_type_name());
    retcode = dds_perception_TrajectoryTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "egoMotion");
    Trajectory_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (Trajectory_topic == NULL)
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

    /* ObjectsDetected topic */
    DDS_Topic *ObjectsDetected_topic;
    strcpy(type_name_tmp, dds_perception_ObjectsDetectedTypeSupport_get_type_name());
    retcode = dds_perception_ObjectsDetectedTypeSupport_register_type(
        application->participant, type_name_tmp);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to register type: %s\n", type_name_tmp);
        goto done;
    }
    strcpy(topic_name_tmp, "detectedObjects");
    ObjectsDetected_topic = DDS_DomainParticipant_create_topic(
        application->participant, topic_name_tmp, type_name_tmp,
        &DDS_TOPIC_QOS_DEFAULT, NULL, DDS_STATUS_MASK_NONE);
    if (ObjectsDetected_topic == NULL)
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
    ObjectsDetected_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    ObjectsDetected_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    ObjectsDetected_dw_qos.resource_limits.max_samples_per_instance = 32;
    ObjectsDetected_dw_qos.resource_limits.max_instances = 2;
    ObjectsDetected_dw_qos.resource_limits.max_samples = ObjectsDetected_dw_qos.resource_limits.max_instances *
        ObjectsDetected_dw_qos.resource_limits.max_samples_per_instance;
    ObjectsDetected_dw_qos.history.depth = 32;
    ObjectsDetected_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    ObjectsDetected_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    ObjectsDetected_dw_listener.on_publication_matched = dds_perception_ObjectsDetectedPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    ObjectsDetected_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        ObjectsDetected_topic,
        &ObjectsDetected_dw_qos,
        &ObjectsDetected_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (ObjectsDetected_datawriter == NULL)
    {
        printf("ObjectsDetected_datawriter == NULL\n");
        goto done;
    }

    ObjectsDetected_hw_datawriter = dds_perception_ObjectsDetectedDataWriter_narrow(ObjectsDetected_datawriter);
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
    * filtered so only one instance is needed.  */
    Lidar_dr_qos.resource_limits.max_instances = 2;
    Lidar_dr_qos.resource_limits.max_samples_per_instance = 32;
    Lidar_dr_qos.resource_limits.max_samples = Lidar_dr_qos.resource_limits.max_instances *
        Lidar_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    Lidar_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Lidar_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Lidar_dr_qos.history.depth = 32;

    Camera_dr_qos.resource_limits.max_instances = 2;
    Camera_dr_qos.resource_limits.max_samples_per_instance = 32;
    Camera_dr_qos.resource_limits.max_samples = Camera_dr_qos.resource_limits.max_instances *
        Camera_dr_qos.resource_limits.max_samples_per_instance;
    Camera_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Camera_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Camera_dr_qos.history.depth = 32;

    Pose3D_real_dr_qos.resource_limits.max_instances = 2;
    Pose3D_real_dr_qos.resource_limits.max_samples_per_instance = 32;
    Pose3D_real_dr_qos.resource_limits.max_samples = Pose3D_real_dr_qos.resource_limits.max_instances *
        Pose3D_real_dr_qos.resource_limits.max_samples_per_instance;
    Pose3D_real_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Pose3D_real_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Pose3D_real_dr_qos.history.depth = 32;

    Trajectory_dr_qos.resource_limits.max_instances = 2;
    Trajectory_dr_qos.resource_limits.max_samples_per_instance = 32;
    Trajectory_dr_qos.resource_limits.max_samples = Trajectory_dr_qos.resource_limits.max_instances *
        Trajectory_dr_qos.resource_limits.max_samples_per_instance;
    Trajectory_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Trajectory_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Trajectory_dr_qos.history.depth = 32;

    MapData_dr_qos.resource_limits.max_instances = 2;
    MapData_dr_qos.resource_limits.max_samples_per_instance = 32;
    MapData_dr_qos.resource_limits.max_samples = MapData_dr_qos.resource_limits.max_instances *
        MapData_dr_qos.resource_limits.max_samples_per_instance;
    MapData_dr_qos.reader_resource_limits.max_remote_writers = 10;
    MapData_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    MapData_dr_qos.history.depth = 32;

    Lidar_dr_listener.on_data_available = dds_sensing_LidarSubscriber_on_data_available;
    Lidar_dr_listener.on_subscription_matched =
        dds_sensing_LidarSubscriber_on_subscription_matched;
    Lidar_dr_listener.as_listener.listener_data = &total_samples;
    Lidar_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Lidar_topic),
        &Lidar_dr_qos,
        &Lidar_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (Lidar_datareader == NULL)
    {
        printf("Lidar_datareader == NULL\n");
        goto done;
    }

    Camera_dr_listener.on_data_available = dds_sensing_CameraSubscriber_on_data_available;
    Camera_dr_listener.on_subscription_matched =
        dds_sensing_CameraSubscriber_on_subscription_matched;
    Camera_dr_listener.as_listener.listener_data = &total_samples;
    Camera_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Camera_topic),
        &Camera_dr_qos,
        &Camera_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (Camera_datareader == NULL)
    {
        printf("Camera_datareader == NULL\n");
        goto done;
    }

    Pose3D_real_dr_listener.on_data_available = dds_physics_Pose3D_realSubscriber_on_data_available;
    Pose3D_real_dr_listener.on_subscription_matched =
        dds_physics_Pose3D_realSubscriber_on_subscription_matched;
    Pose3D_real_dr_listener.as_listener.listener_data = &total_samples;
    Pose3D_real_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Pose3D_real_topic),
        &Pose3D_real_dr_qos,
        &Pose3D_real_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (Pose3D_real_datareader == NULL)
    {
        printf("Pose3D_real_datareader == NULL\n");
        goto done;
    }

    Trajectory_dr_listener.on_data_available = dds_perception_TrajectorySubscriber_on_data_available;
    Trajectory_dr_listener.on_subscription_matched =
        dds_perception_TrajectorySubscriber_on_subscription_matched;
    Trajectory_dr_listener.as_listener.listener_data = &total_samples;
    Trajectory_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Trajectory_topic),
        &Trajectory_dr_qos,
        &Trajectory_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (Trajectory_datareader == NULL)
    {
        printf("Trajectory_datareader == NULL\n");
        goto done;
    }

    MapData_dr_listener.on_data_available = dds_planning_MapDataSubscriber_on_data_available;
    MapData_dr_listener.on_subscription_matched =
        dds_planning_MapDataSubscriber_on_subscription_matched;
    MapData_dr_listener.as_listener.listener_data = &total_samples;
    MapData_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(MapData_topic),
        &MapData_dr_qos,
        &MapData_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);
    if (MapData_datareader == NULL)
    {
        printf("MapData_datareader == NULL\n");
        goto done;
    }

#ifdef RTI_CERT
#ifdef RTI_VXWORKS
    /** End initialization, disable further dynamic memory allocation ***/
    memAllocDisable();
#endif  /* RTI_VXWORKS */
#endif  /* RTI_CERT */

    /* Initialize values in the pub topics */
    ObjectsDetected_sample->parent.id[0] = 0002;

    /* loop ---------------- */
    for (i = 0; (application->count <= 0) || (i < application->count); ++i)
    {
        /* TODO set *_sample values/attributes here */
        ObjectsDetected_sample->test = i;

        retcode = dds_perception_ObjectsDetectedDataWriter_write(
            ObjectsDetected_hw_datawriter,
            ObjectsDetected_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "ObjectsDetected");
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

    if (ObjectsDetected_sample != NULL)
    {
        dds_perception_ObjectsDetectedTypeSupport_delete_data(ObjectsDetected_sample);
    }

#endif  /* RTI_CERT */
#ifndef RTI_CERT
    retcode = DDS_DataWriterQos_finalize(&ObjectsDetected_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Lidar_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Camera_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Pose3D_real_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Trajectory_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&MapData_dr_qos);
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
