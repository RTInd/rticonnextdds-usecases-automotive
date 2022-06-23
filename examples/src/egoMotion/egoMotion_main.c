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
/* egoMotion_main.c
* An application skeleton using the following DDS data types:
*   dds::perception::ObjectsDetected
*   dds::sensing::gnss::Basic
*   dds::sensing::imu::Imu_real
*   dds::sensing::Compass
*   dds::perception::Trajectory
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
#include "res/types/data/perception/Object_tSupport.h"
#include "res/types/data/sensing/Gnss_tSupport.h"
#include "res/types/data/sensing/Imu_tSupport.h"
#include "res/types/data/sensing/Compass_tSupport.h"
#include "res/types/data/perception/Trajectory_tSupport.h"
#include "egoMotion_dp.h"

/* DATA WRITER LISTENERS: called when pub is matched to a subscriber */
void
dds_perception_TrajectoryPublisher_on_publication_matched(
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
dds_perception_ObjectsDetectedSubscriber_on_subscription_matched(
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
dds_perception_ObjectsDetectedSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_perception_ObjectsDetectedDataReader *hw_reader = dds_perception_ObjectsDetectedDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_perception_ObjectsDetected *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_perception_ObjectsDetectedSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_perception_ObjectsDetectedDataReader_take(
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
    for (i = 0; i < dds_perception_ObjectsDetectedSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_perception_ObjectsDetectedSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_perception_ObjectsDetectedDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_perception_ObjectsDetectedSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}


void
dds_sensing_gnss_BasicSubscriber_on_subscription_matched(
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
dds_sensing_gnss_BasicSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_sensing_gnss_BasicDataReader *hw_reader = dds_sensing_gnss_BasicDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_sensing_gnss_Basic *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_sensing_gnss_BasicSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_sensing_gnss_BasicDataReader_take(
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
    for (i = 0; i < dds_sensing_gnss_BasicSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_sensing_gnss_BasicSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_sensing_gnss_BasicDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_sensing_gnss_BasicSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_sensing_imu_Imu_realSubscriber_on_subscription_matched(
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
dds_sensing_imu_Imu_realSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_sensing_imu_Imu_realDataReader *hw_reader = dds_sensing_imu_Imu_realDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_sensing_imu_Imu_real *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_sensing_imu_Imu_realSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_sensing_imu_Imu_realDataReader_take(
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
    for (i = 0; i < dds_sensing_imu_Imu_realSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_sensing_imu_Imu_realSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_sensing_imu_Imu_realDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_sensing_imu_Imu_realSeq_finalize(&sample_seq);
    DDS_SampleInfoSeq_finalize(&info_seq);
#else  /* RTI_CERT */
    return;
#endif  /* RTI_CERT */
}

void
dds_sensing_CompassSubscriber_on_subscription_matched(
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
dds_sensing_CompassSubscriber_on_data_available(
    void *listener_data,
    DDS_DataReader * reader)
{
    dds_sensing_CompassDataReader *hw_reader = dds_sensing_CompassDataReader_narrow(reader);
    DDS_ReturnCode_t retcode;
    struct DDS_SampleInfo *sample_info = NULL;
    dds_sensing_Compass *sample = NULL;

    struct DDS_SampleInfoSeq info_seq =
    DDS_SEQUENCE_INITIALIZER;
    struct dds_sensing_CompassSeq sample_seq =
    DDS_SEQUENCE_INITIALIZER;

    DDS_Long i;
    DDS_Long *total_samples = (DDS_Long*) listener_data;

    retcode = dds_sensing_CompassDataReader_take(
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
    for (i = 0; i < dds_sensing_CompassSeq_get_length(&sample_seq); ++i)
    {
        sample_info = DDS_SampleInfoSeq_get_reference(&info_seq, i);

        if (sample_info->valid_data)
        {
            sample = dds_sensing_CompassSeq_get_reference(&sample_seq, i);
            printf("\nValid sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nSample received\n\tINVALID DATA\n");
        }
    }

    dds_sensing_CompassDataReader_return_loan(hw_reader, &sample_seq, &info_seq);

    done:
#ifndef RTI_CERT
    dds_sensing_CompassSeq_finalize(&sample_seq);
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
    DDS_DataWriter *Trajectory_datawriter;
    dds_perception_TrajectoryDataWriter *Trajectory_hw_datawriter;
    struct DDS_DataWriterQos Trajectory_dw_qos = DDS_DataWriterQos_INITIALIZER;
    dds_perception_Trajectory *Trajectory_sample = NULL;

    DDS_Subscriber *subscriber;
    DDS_DataReader *ObjectsDetected_datareader;
    struct DDS_DataReaderQos ObjectsDetected_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Basic_datareader;
    struct DDS_DataReaderQos Basic_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Imu_real_datareader;
    struct DDS_DataReaderQos Imu_real_dr_qos = DDS_DataReaderQos_INITIALIZER;
    DDS_DataReader *Compass_datareader;
    struct DDS_DataReaderQos Compass_dr_qos = DDS_DataReaderQos_INITIALIZER;

    struct DDS_DataWriterListener Trajectory_dw_listener = DDS_DataWriterListener_INITIALIZER;
    Trajectory_sample = dds_perception_TrajectoryTypeSupport_create_data();
    if (Trajectory_sample == NULL)
    {
        printf("failed dds_perception_TrajectoryTypeSupport_create_data\n");
        return -1;
    }
    struct DDS_DataReaderListener ObjectsDetected_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Basic_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Imu_real_dr_listener = DDS_DataReaderListener_INITIALIZER;
    struct DDS_DataReaderListener Compass_dr_listener = DDS_DataReaderListener_INITIALIZER;
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
    Trajectory_dw_qos.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else  /* USE_RELIABLE_QOS */
    Trajectory_dw_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif  /* USE_RELIABLE_QOS */
    Trajectory_dw_qos.resource_limits.max_samples_per_instance = 32;
    Trajectory_dw_qos.resource_limits.max_instances = 2;
    Trajectory_dw_qos.resource_limits.max_samples = Trajectory_dw_qos.resource_limits.max_instances *
        Trajectory_dw_qos.resource_limits.max_samples_per_instance;
    Trajectory_dw_qos.history.depth = 32;
    Trajectory_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.sec = 0;
    Trajectory_dw_qos.protocol.rtps_reliable_writer.heartbeat_period.nanosec = 250000000;

    Trajectory_dw_listener.on_publication_matched = dds_perception_TrajectoryPublisher_on_publication_matched;

    /* create a datawriter for this topic */
    Trajectory_datawriter = DDS_Publisher_create_datawriter(
        publisher,
        Trajectory_topic,
        &Trajectory_dw_qos,
        &Trajectory_dw_listener,
        DDS_PUBLICATION_MATCHED_STATUS);
    if (Trajectory_datawriter == NULL)
    {
        printf("Trajectory_datawriter == NULL\n");
        goto done;
    }

    Trajectory_hw_datawriter = dds_perception_TrajectoryDataWriter_narrow(Trajectory_datawriter);
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
    ObjectsDetected_dr_qos.resource_limits.max_instances = 2;
    ObjectsDetected_dr_qos.resource_limits.max_samples_per_instance = 32;
    ObjectsDetected_dr_qos.resource_limits.max_samples = ObjectsDetected_dr_qos.resource_limits.max_instances *
        ObjectsDetected_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    ObjectsDetected_dr_qos.reader_resource_limits.max_remote_writers = 10;
    ObjectsDetected_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    ObjectsDetected_dr_qos.history.depth = 32;

    Basic_dr_qos.resource_limits.max_instances = 2;
    Basic_dr_qos.resource_limits.max_samples_per_instance = 32;
    Basic_dr_qos.resource_limits.max_samples = Basic_dr_qos.resource_limits.max_instances *
        Basic_dr_qos.resource_limits.max_samples_per_instance;
    Basic_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Basic_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Basic_dr_qos.history.depth = 32;

    Imu_real_dr_qos.resource_limits.max_instances = 2;
    Imu_real_dr_qos.resource_limits.max_samples_per_instance = 32;
    Imu_real_dr_qos.resource_limits.max_samples = Imu_real_dr_qos.resource_limits.max_instances *
        Imu_real_dr_qos.resource_limits.max_samples_per_instance;
    Imu_real_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Imu_real_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Imu_real_dr_qos.history.depth = 32;

    Compass_dr_qos.resource_limits.max_instances = 2;
    Compass_dr_qos.resource_limits.max_samples_per_instance = 32;
    Compass_dr_qos.resource_limits.max_samples = Compass_dr_qos.resource_limits.max_instances *
        Compass_dr_qos.resource_limits.max_samples_per_instance;
    Compass_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Compass_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Compass_dr_qos.history.depth = 32;

    ObjectsDetected_dr_listener.on_data_available = dds_perception_ObjectsDetectedSubscriber_on_data_available;
    ObjectsDetected_dr_listener.on_subscription_matched =
        dds_perception_ObjectsDetectedSubscriber_on_subscription_matched;

    ObjectsDetected_dr_listener.as_listener.listener_data = &total_samples;

    ObjectsDetected_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(ObjectsDetected_topic),
        &ObjectsDetected_dr_qos,
        &ObjectsDetected_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);

    if (ObjectsDetected_datareader == NULL)
    {
        printf("ObjectsDetected_datareader == NULL\n");
        goto done;
    }

    Basic_dr_listener.on_data_available = dds_sensing_gnss_BasicSubscriber_on_data_available;
    Basic_dr_listener.on_subscription_matched =
        dds_sensing_gnss_BasicSubscriber_on_subscription_matched;

    Basic_dr_listener.as_listener.listener_data = &total_samples;

    Basic_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Basic_topic),
        &Basic_dr_qos,
        &Basic_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);

    if (Basic_datareader == NULL)
    {
        printf("Basic_datareader == NULL\n");
        goto done;
    }

    Imu_real_dr_listener.on_data_available = dds_sensing_imu_Imu_realSubscriber_on_data_available;
    Imu_real_dr_listener.on_subscription_matched =
        dds_sensing_imu_Imu_realSubscriber_on_subscription_matched;

    Imu_real_dr_listener.as_listener.listener_data = &total_samples;

    Imu_real_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Imu_real_topic),
        &Imu_real_dr_qos,
        &Imu_real_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);

    if (Imu_real_datareader == NULL)
    {
        printf("Imu_real_datareader == NULL\n");
        goto done;
    }

    Compass_dr_listener.on_data_available = dds_sensing_CompassSubscriber_on_data_available;
    Compass_dr_listener.on_subscription_matched =
        dds_sensing_CompassSubscriber_on_subscription_matched;

    Compass_dr_listener.as_listener.listener_data = &total_samples;

    Compass_datareader = DDS_Subscriber_create_datareader(
        subscriber,
        DDS_Topic_as_topicdescription(Compass_topic),
        &Compass_dr_qos,
        &Compass_dr_listener,
        DDS_DATA_AVAILABLE_STATUS | DDS_SUBSCRIPTION_MATCHED_STATUS);

    if (Compass_datareader == NULL)
    {
        printf("Compass_datareader == NULL\n");
        goto done;
    }

#ifdef RTI_CERT
#ifdef RTI_VXWORKS
    /** End initialization, disable further dynamic memory allocation ***/
    memAllocDisable();
#endif  /* RTI_VXWORKS */
#endif  /* RTI_CERT */

    /* loop ---------------- */
    for (i = 0; (application->count <= 0) || (i < application->count); ++i)
    {
        /* TODO set *_sample values/attributes here */

        retcode = dds_perception_TrajectoryDataWriter_write(
            Trajectory_hw_datawriter,
            Trajectory_sample,
            &DDS_HANDLE_NIL);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("Failed to write %s sample\n", "Trajectory");
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

    if (Trajectory_sample != NULL)
    {
        dds_perception_TrajectoryTypeSupport_delete_data(Trajectory_sample);
    }

#endif  /* RTI_CERT */
#ifndef RTI_CERT
    retcode = DDS_DataWriterQos_finalize(&Trajectory_dw_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataWriterQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&ObjectsDetected_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Basic_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Imu_real_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    retcode = DDS_DataReaderQos_finalize(&Compass_dr_qos);
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
