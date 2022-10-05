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
#include "disc_dpse/disc_dpse_dpsediscovery.h"

/* DATA WRITER LISTENERS: called when pub is matched to a subscriber */
void
dds_perception_ObjectsDetectedPublisher_on_publication_matched(
    void *listener_data,
    DDS_DataWriter *writer,
    const struct DDS_PublicationMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a ObjectsDetected subscriber\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a ObjectDetected subscriber\n");
    }
}

#ifdef USE_SAMPLE_FILTER
#ifdef FILTER_ON_DESERIALIZE

/* See Wire Protocol Specification on http://www.omg.org/spec/DDSI-RTPS/2.2/
* for more details about CDR encapsulation.
*/


/*i
* \brief Helper function to deserialize an unsigned long
*
* \param[in]  src_buffer      Pointer to CDR stream buffer
* \param[in]  need_byte_swap  Indicates whether it is needed to swap bytes
* \param[out] instance        Deserialized unsigned long
*/
void
dds_sensing_LidarSubscriber_deserialize_unsigned_long(
    char **src_buffer,
    RTI_BOOL need_byte_swap,
    DDS_UnsignedLong *instance)
{
    RTI_INT32 i;
    if (need_byte_swap)
    {
        for (i = 3; i >= 0; --i)
        {
            *((RTI_INT8*)instance + i) = *((*src_buffer)++);
        }
    }
    else
    {
        *instance = *(RTI_UINT32*)(*src_buffer);
        (*src_buffer) += CDR_LONG_SIZE;
    }
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_deserialize
*/
DDS_Boolean
dds_sensing_LidarSubscriber_on_before_sample_deserialize(
    void *listener_data,
    DDS_DataReader *reader,
    struct NDDS_Type_Plugin *plugin,
    struct CDR_Stream_t *stream,
    DDS_Boolean *dropped)
{
    DDS_Boolean result = DDS_BOOLEAN_FALSE;
    DDS_Long id = 0;
    RTI_BOOL need_byte_swap = DDS_BOOLEAN_FALSE;
    char *src_buffer = NULL;

    need_byte_swap = CDR_Stream_is_byte_swapped(stream);
    src_buffer = CDR_Stream_get_current_position_ptr(stream);

    /* deserialize only field 'id', instead of the whole sample */

    if (!CDR_Stream_check_size(stream, UNSIGNED_LONG_SIZE))
    {
        printf("Failed to deserialize id. The stream is too short, missing data\n");
        goto done;
    }

    /* Note primitive types must be aligned to their length in the CDR stream.
    * For example, a long must start on a 4-byte boundary. The boundaries are
    * counted from the start of the CDR stream.
    * As the sample 'id' is the first data in the stream it is already aligned.
    * Position 0 (beginning of the stream) is aligned to 4 (size of long).
    *
    * NOTE: If you want to use a different field for filtering (e.g. you type does
    * not have a field called id as first field), you will need to reimplement this
    * function and dds_sensing_LidarSubscriber_deserialize_unsigned_long
    * to match your type.
    */
    dds_sensing_LidarSubscriber_deserialize_unsigned_long(
        &src_buffer,
        need_byte_swap,
        (DDS_UnsignedLong*)&id);

    /* filter the sample */
    *dropped = (id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    if (*dropped)
    {
        printf("\nSample filtered, before deserialize...\n\tDROPPED - id: %d\n", id);
    }

    result = DDS_BOOLEAN_TRUE;
    done:
    return result;
}

#else

/*i
* \brief Helper function to filter an dds_sensing_Lidar sample
*
* \param[in]  sample       A dds_sensing_Lidar data sample to filter
* \param[out] drop_sample  Out parameter determining whether the sample
*                          should be filtered out or not.
*/
void
dds_sensing_LidarSubscriber_filter_sample(
    dds_sensing_Lidar *sample,
    DDS_Boolean *drop_sample)
{
    /* Example filter: drop samples with even-numbered count in id */
    /* NOTE: If field "id" does not exist in your data type, change "id"
    * by the field you want to use for filtering.
    */
    *drop_sample = (sample->id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_commit
*/
DDS_Boolean
dds_sensing_LidarSubscriber_on_before_sample_commit(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    dds_sensing_Lidar *hw_sample = (dds_sensing_Lidar *)sample;

    dds_sensing_LidarSubscriber_filter_sample(hw_sample, dropped);

    if (*dropped)
    {
        printf("\nSample filtered, before commit...\n");
    }

    return DDS_BOOLEAN_TRUE;
}
#endif /* FILTER_ON_DESERIALIZE */
#endif /* USE_SAMPLE_FILTER */

void
dds_sensing_LidarSubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a Lidar publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a Lidar publisher\n");
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
            printf("Valid Lidar_sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nLidar_sample received\n\tINVALID DATA\n");
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

#ifdef USE_SAMPLE_FILTER
#ifdef FILTER_ON_DESERIALIZE

/* See Wire Protocol Specification on http://www.omg.org/spec/DDSI-RTPS/2.2/
* for more details about CDR encapsulation.
*/


/*i
* \brief Helper function to deserialize an unsigned long
*
* \param[in]  src_buffer      Pointer to CDR stream buffer
* \param[in]  need_byte_swap  Indicates whether it is needed to swap bytes
* \param[out] instance        Deserialized unsigned long
*/
void
dds_sensing_CameraSubscriber_deserialize_unsigned_long(
    char **src_buffer,
    RTI_BOOL need_byte_swap,
    DDS_UnsignedLong *instance)
{
    RTI_INT32 i;
    if (need_byte_swap)
    {
        for (i = 3; i >= 0; --i)
        {
            *((RTI_INT8*)instance + i) = *((*src_buffer)++);
        }
    }
    else
    {
        *instance = *(RTI_UINT32*)(*src_buffer);
        (*src_buffer) += CDR_LONG_SIZE;
    }
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_deserialize
*/
DDS_Boolean
dds_sensing_CameraSubscriber_on_before_sample_deserialize(
    void *listener_data,
    DDS_DataReader *reader,
    struct NDDS_Type_Plugin *plugin,
    struct CDR_Stream_t *stream,
    DDS_Boolean *dropped)
{
    DDS_Boolean result = DDS_BOOLEAN_FALSE;
    DDS_Long id = 0;
    RTI_BOOL need_byte_swap = DDS_BOOLEAN_FALSE;
    char *src_buffer = NULL;

    need_byte_swap = CDR_Stream_is_byte_swapped(stream);
    src_buffer = CDR_Stream_get_current_position_ptr(stream);

    /* deserialize only field 'id', instead of the whole sample */

    if (!CDR_Stream_check_size(stream, UNSIGNED_LONG_SIZE))
    {
        printf("Failed to deserialize id. The stream is too short, missing data\n");
        goto done;
    }

    /* Note primitive types must be aligned to their length in the CDR stream.
    * For example, a long must start on a 4-byte boundary. The boundaries are
    * counted from the start of the CDR stream.
    * As the sample 'id' is the first data in the stream it is already aligned.
    * Position 0 (beginning of the stream) is aligned to 4 (size of long).
    *
    * NOTE: If you want to use a different field for filtering (e.g. you type does
    * not have a field called id as first field), you will need to reimplement this
    * function and dds_sensing_CameraSubscriber_deserialize_unsigned_long
    * to match your type.
    */
    dds_sensing_CameraSubscriber_deserialize_unsigned_long(
        &src_buffer,
        need_byte_swap,
        (DDS_UnsignedLong*)&id);

    /* filter the sample */
    *dropped = (id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    if (*dropped)
    {
        printf("\nSample filtered, before deserialize...\n\tDROPPED - id: %d\n", id);
    }

    result = DDS_BOOLEAN_TRUE;
    done:
    return result;
}

#else

/*i
* \brief Helper function to filter an dds_sensing_Camera sample
*
* \param[in]  sample       A dds_sensing_Camera data sample to filter
* \param[out] drop_sample  Out parameter determining whether the sample
*                          should be filtered out or not.
*/
void
dds_sensing_CameraSubscriber_filter_sample(
    dds_sensing_Camera *sample,
    DDS_Boolean *drop_sample)
{
    /* Example filter: drop samples with even-numbered count in id */
    /* NOTE: If field "id" does not exist in your data type, change "id"
    * by the field you want to use for filtering.
    */
    *drop_sample = (sample->id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_commit
*/
DDS_Boolean
dds_sensing_CameraSubscriber_on_before_sample_commit(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    dds_sensing_Camera *hw_sample = (dds_sensing_Camera *)sample;

    dds_sensing_CameraSubscriber_filter_sample(hw_sample, dropped);

    if (*dropped)
    {
        printf("\nSample filtered, before commit...\n");
    }

    return DDS_BOOLEAN_TRUE;
}
#endif /* FILTER_ON_DESERIALIZE */
#endif /* USE_SAMPLE_FILTER */

void
dds_sensing_CameraSubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a Camera publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a Camera publisher\n");
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
            printf("Valid Camera_sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nCamera_sample received\n\tINVALID DATA\n");
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

#ifdef USE_SAMPLE_FILTER
#ifdef FILTER_ON_DESERIALIZE

/* See Wire Protocol Specification on http://www.omg.org/spec/DDSI-RTPS/2.2/
* for more details about CDR encapsulation.
*/


/*i
* \brief Helper function to deserialize an unsigned long
*
* \param[in]  src_buffer      Pointer to CDR stream buffer
* \param[in]  need_byte_swap  Indicates whether it is needed to swap bytes
* \param[out] instance        Deserialized unsigned long
*/
void
dds_physics_Pose3D_realSubscriber_deserialize_unsigned_long(
    char **src_buffer,
    RTI_BOOL need_byte_swap,
    DDS_UnsignedLong *instance)
{
    RTI_INT32 i;
    if (need_byte_swap)
    {
        for (i = 3; i >= 0; --i)
        {
            *((RTI_INT8*)instance + i) = *((*src_buffer)++);
        }
    }
    else
    {
        *instance = *(RTI_UINT32*)(*src_buffer);
        (*src_buffer) += CDR_LONG_SIZE;
    }
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_deserialize
*/
DDS_Boolean
dds_physics_Pose3D_realSubscriber_on_before_sample_deserialize(
    void *listener_data,
    DDS_DataReader *reader,
    struct NDDS_Type_Plugin *plugin,
    struct CDR_Stream_t *stream,
    DDS_Boolean *dropped)
{
    DDS_Boolean result = DDS_BOOLEAN_FALSE;
    DDS_Long id = 0;
    RTI_BOOL need_byte_swap = DDS_BOOLEAN_FALSE;
    char *src_buffer = NULL;

    need_byte_swap = CDR_Stream_is_byte_swapped(stream);
    src_buffer = CDR_Stream_get_current_position_ptr(stream);

    /* deserialize only field 'id', instead of the whole sample */

    if (!CDR_Stream_check_size(stream, UNSIGNED_LONG_SIZE))
    {
        printf("Failed to deserialize id. The stream is too short, missing data\n");
        goto done;
    }

    /* Note primitive types must be aligned to their length in the CDR stream.
    * For example, a long must start on a 4-byte boundary. The boundaries are
    * counted from the start of the CDR stream.
    * As the sample 'id' is the first data in the stream it is already aligned.
    * Position 0 (beginning of the stream) is aligned to 4 (size of long).
    *
    * NOTE: If you want to use a different field for filtering (e.g. you type does
    * not have a field called id as first field), you will need to reimplement this
    * function and dds_physics_Pose3D_realSubscriber_deserialize_unsigned_long
    * to match your type.
    */
    dds_physics_Pose3D_realSubscriber_deserialize_unsigned_long(
        &src_buffer,
        need_byte_swap,
        (DDS_UnsignedLong*)&id);

    /* filter the sample */
    *dropped = (id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    if (*dropped)
    {
        printf("\nSample filtered, before deserialize...\n\tDROPPED - id: %d\n", id);
    }

    result = DDS_BOOLEAN_TRUE;
    done:
    return result;
}

#else

/*i
* \brief Helper function to filter an dds_physics_Pose3D_real sample
*
* \param[in]  sample       A dds_physics_Pose3D_real data sample to filter
* \param[out] drop_sample  Out parameter determining whether the sample
*                          should be filtered out or not.
*/
void
dds_physics_Pose3D_realSubscriber_filter_sample(
    dds_physics_Pose3D_real *sample,
    DDS_Boolean *drop_sample)
{
    /* Example filter: drop samples with even-numbered count in id */
    /* NOTE: If field "id" does not exist in your data type, change "id"
    * by the field you want to use for filtering.
    */
    *drop_sample = (sample->id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_commit
*/
DDS_Boolean
dds_physics_Pose3D_realSubscriber_on_before_sample_commit(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    dds_physics_Pose3D_real *hw_sample = (dds_physics_Pose3D_real *)sample;

    dds_physics_Pose3D_realSubscriber_filter_sample(hw_sample, dropped);

    if (*dropped)
    {
        printf("\nSample filtered, before commit...\n");
    }

    return DDS_BOOLEAN_TRUE;
}
#endif /* FILTER_ON_DESERIALIZE */
#endif /* USE_SAMPLE_FILTER */

void
dds_physics_Pose3D_realSubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a Pose3D publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a Pose3D publisher\n");
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
            printf("Valid Pose3D_sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nPose3D_sample received\n\tINVALID DATA\n");
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

#ifdef USE_SAMPLE_FILTER
#ifdef FILTER_ON_DESERIALIZE

/* See Wire Protocol Specification on http://www.omg.org/spec/DDSI-RTPS/2.2/
* for more details about CDR encapsulation.
*/


/*i
* \brief Helper function to deserialize an unsigned long
*
* \param[in]  src_buffer      Pointer to CDR stream buffer
* \param[in]  need_byte_swap  Indicates whether it is needed to swap bytes
* \param[out] instance        Deserialized unsigned long
*/
void
dds_perception_TrajectorySubscriber_deserialize_unsigned_long(
    char **src_buffer,
    RTI_BOOL need_byte_swap,
    DDS_UnsignedLong *instance)
{
    RTI_INT32 i;
    if (need_byte_swap)
    {
        for (i = 3; i >= 0; --i)
        {
            *((RTI_INT8*)instance + i) = *((*src_buffer)++);
        }
    }
    else
    {
        *instance = *(RTI_UINT32*)(*src_buffer);
        (*src_buffer) += CDR_LONG_SIZE;
    }
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_deserialize
*/
DDS_Boolean
dds_perception_TrajectorySubscriber_on_before_sample_deserialize(
    void *listener_data,
    DDS_DataReader *reader,
    struct NDDS_Type_Plugin *plugin,
    struct CDR_Stream_t *stream,
    DDS_Boolean *dropped)
{
    DDS_Boolean result = DDS_BOOLEAN_FALSE;
    DDS_Long id = 0;
    RTI_BOOL need_byte_swap = DDS_BOOLEAN_FALSE;
    char *src_buffer = NULL;

    need_byte_swap = CDR_Stream_is_byte_swapped(stream);
    src_buffer = CDR_Stream_get_current_position_ptr(stream);

    /* deserialize only field 'id', instead of the whole sample */

    if (!CDR_Stream_check_size(stream, UNSIGNED_LONG_SIZE))
    {
        printf("Failed to deserialize id. The stream is too short, missing data\n");
        goto done;
    }

    /* Note primitive types must be aligned to their length in the CDR stream.
    * For example, a long must start on a 4-byte boundary. The boundaries are
    * counted from the start of the CDR stream.
    * As the sample 'id' is the first data in the stream it is already aligned.
    * Position 0 (beginning of the stream) is aligned to 4 (size of long).
    *
    * NOTE: If you want to use a different field for filtering (e.g. you type does
    * not have a field called id as first field), you will need to reimplement this
    * function and dds_perception_TrajectorySubscriber_deserialize_unsigned_long
    * to match your type.
    */
    dds_perception_TrajectorySubscriber_deserialize_unsigned_long(
        &src_buffer,
        need_byte_swap,
        (DDS_UnsignedLong*)&id);

    /* filter the sample */
    *dropped = (id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    if (*dropped)
    {
        printf("\nSample filtered, before deserialize...\n\tDROPPED - id: %d\n", id);
    }

    result = DDS_BOOLEAN_TRUE;
    done:
    return result;
}

#else

/*i
* \brief Helper function to filter an dds_perception_Trajectory sample
*
* \param[in]  sample       A dds_perception_Trajectory data sample to filter
* \param[out] drop_sample  Out parameter determining whether the sample
*                          should be filtered out or not.
*/
void
dds_perception_TrajectorySubscriber_filter_sample(
    dds_perception_Trajectory *sample,
    DDS_Boolean *drop_sample)
{
    /* Example filter: drop samples with even-numbered count in id */
    /* NOTE: If field "id" does not exist in your data type, change "id"
    * by the field you want to use for filtering.
    */
    *drop_sample = (sample->id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_commit
*/
DDS_Boolean
dds_perception_TrajectorySubscriber_on_before_sample_commit(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    dds_perception_Trajectory *hw_sample = (dds_perception_Trajectory *)sample;

    dds_perception_TrajectorySubscriber_filter_sample(hw_sample, dropped);

    if (*dropped)
    {
        printf("\nSample filtered, before commit...\n");
    }

    return DDS_BOOLEAN_TRUE;
}
#endif /* FILTER_ON_DESERIALIZE */
#endif /* USE_SAMPLE_FILTER */

void
dds_perception_TrajectorySubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a Trajectory publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a Trajectory publisher\n");
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
            printf("Valid Trajectory_sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nTrajectory_sample received\n\tINVALID DATA\n");
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

#ifdef USE_SAMPLE_FILTER
#ifdef FILTER_ON_DESERIALIZE

/* See Wire Protocol Specification on http://www.omg.org/spec/DDSI-RTPS/2.2/
* for more details about CDR encapsulation.
*/


/*i
* \brief Helper function to deserialize an unsigned long
*
* \param[in]  src_buffer      Pointer to CDR stream buffer
* \param[in]  need_byte_swap  Indicates whether it is needed to swap bytes
* \param[out] instance        Deserialized unsigned long
*/
void
dds_planning_MapDataSubscriber_deserialize_unsigned_long(
    char **src_buffer,
    RTI_BOOL need_byte_swap,
    DDS_UnsignedLong *instance)
{
    RTI_INT32 i;
    if (need_byte_swap)
    {
        for (i = 3; i >= 0; --i)
        {
            *((RTI_INT8*)instance + i) = *((*src_buffer)++);
        }
    }
    else
    {
        *instance = *(RTI_UINT32*)(*src_buffer);
        (*src_buffer) += CDR_LONG_SIZE;
    }
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_deserialize
*/
DDS_Boolean
dds_planning_MapDataSubscriber_on_before_sample_deserialize(
    void *listener_data,
    DDS_DataReader *reader,
    struct NDDS_Type_Plugin *plugin,
    struct CDR_Stream_t *stream,
    DDS_Boolean *dropped)
{
    DDS_Boolean result = DDS_BOOLEAN_FALSE;
    DDS_Long id = 0;
    RTI_BOOL need_byte_swap = DDS_BOOLEAN_FALSE;
    char *src_buffer = NULL;

    need_byte_swap = CDR_Stream_is_byte_swapped(stream);
    src_buffer = CDR_Stream_get_current_position_ptr(stream);

    /* deserialize only field 'id', instead of the whole sample */

    if (!CDR_Stream_check_size(stream, UNSIGNED_LONG_SIZE))
    {
        printf("Failed to deserialize id. The stream is too short, missing data\n");
        goto done;
    }

    /* Note primitive types must be aligned to their length in the CDR stream.
    * For example, a long must start on a 4-byte boundary. The boundaries are
    * counted from the start of the CDR stream.
    * As the sample 'id' is the first data in the stream it is already aligned.
    * Position 0 (beginning of the stream) is aligned to 4 (size of long).
    *
    * NOTE: If you want to use a different field for filtering (e.g. you type does
    * not have a field called id as first field), you will need to reimplement this
    * function and dds_planning_MapDataSubscriber_deserialize_unsigned_long
    * to match your type.
    */
    dds_planning_MapDataSubscriber_deserialize_unsigned_long(
        &src_buffer,
        need_byte_swap,
        (DDS_UnsignedLong*)&id);

    /* filter the sample */
    *dropped = (id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;

    if (*dropped)
    {
        printf("\nSample filtered, before deserialize...\n\tDROPPED - id: %d\n", id);
    }

    result = DDS_BOOLEAN_TRUE;
    done:
    return result;
}

#else

/*i
* \brief Helper function to filter an dds_planning_MapData sample
*
* \param[in]  sample       A dds_planning_MapData data sample to filter
* \param[out] drop_sample  Out parameter determining whether the sample
*                          should be filtered out or not.
*/
void
dds_planning_MapDataSubscriber_filter_sample(
    dds_planning_MapData *sample,
    DDS_Boolean *drop_sample)
{
    /* Example filter: drop samples with even-numbered count in id */
    /* NOTE: If field "id" does not exist in your data type, change "id"
    * by the field you want to use for filtering.
    */
    *drop_sample = (sample->id % 2 == 0) ? DDS_BOOLEAN_TRUE : DDS_BOOLEAN_FALSE;
}

/*i
* \brief Implementation of \ref DDS_DataReaderListener::on_before_sample_commit
*/
DDS_Boolean
dds_planning_MapDataSubscriber_on_before_sample_commit(
    void *listener_data,
    DDS_DataReader *reader,
    const void *const sample,
    const struct DDS_SampleInfo *const sample_info,
    DDS_Boolean *dropped)
{
    dds_planning_MapData *hw_sample = (dds_planning_MapData *)sample;

    dds_planning_MapDataSubscriber_filter_sample(hw_sample, dropped);

    if (*dropped)
    {
        printf("\nSample filtered, before commit...\n");
    }

    return DDS_BOOLEAN_TRUE;
}
#endif /* FILTER_ON_DESERIALIZE */
#endif /* USE_SAMPLE_FILTER */

void
dds_planning_MapDataSubscriber_on_subscription_matched(
    void *listener_data,
    DDS_DataReader *reader,
    const struct DDS_SubscriptionMatchedStatus *status)
{
    if (status->current_count_change > 0)
    {
        printf("Matched a MapData publisher\n");
    }
    else if (status->current_count_change < 0)
    {
        printf("Unmatched a MapData publisher\n");
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
            printf("Valid MapData_sample received\n");
            *total_samples += 1;

            /* TODO read and process sample attributes here */

        }
        else
        {
            printf("\nMapData_sample received\n\tINVALID DATA\n");
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
    struct DDS_SubscriptionBuiltinTopicData ObjectsDetected_rem_subscription_data =
        DDS_SubscriptionBuiltinTopicData_INITIALIZER;
    DDS_Subscriber *subscriber;
    DDS_DataReader *Lidar_datareader;
    struct DDS_DataReaderQos Lidar_dr_qos = DDS_DataReaderQos_INITIALIZER;
    struct DDS_PublicationBuiltinTopicData Lidar_rem_publication_data =
        DDS_PublicationBuiltinTopicData_INITIALIZER;
    DDS_DataReader *Camera_datareader;
    struct DDS_DataReaderQos Camera_dr_qos = DDS_DataReaderQos_INITIALIZER;
    struct DDS_PublicationBuiltinTopicData Camera_rem_publication_data =
        DDS_PublicationBuiltinTopicData_INITIALIZER;
    DDS_DataReader *Pose3D_real_datareader;
    struct DDS_DataReaderQos Pose3D_real_dr_qos = DDS_DataReaderQos_INITIALIZER;
    struct DDS_PublicationBuiltinTopicData Pose3D_real_rem_publication_data =
        DDS_PublicationBuiltinTopicData_INITIALIZER;
    DDS_DataReader *Trajectory_datareader;
    struct DDS_DataReaderQos Trajectory_dr_qos = DDS_DataReaderQos_INITIALIZER;
    struct DDS_PublicationBuiltinTopicData Trajectory_rem_publication_data =
        DDS_PublicationBuiltinTopicData_INITIALIZER;
    DDS_DataReader *MapData_datareader;
    struct DDS_DataReaderQos MapData_dr_qos = DDS_DataReaderQos_INITIALIZER;
    struct DDS_PublicationBuiltinTopicData MapData_rem_publication_data =
        DDS_PublicationBuiltinTopicData_INITIALIZER;

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
    application = Application_create("perception_dp", domain_id, udp_intf, peer);
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

    /* assert for each remote participant to be connected */
    retcode = DPSE_RemoteParticipant_assert(
        application->participant, "localization_dp");     /* Note: this string should match the remote participant name */
    if (retcode != DDS_RETCODE_OK)  {
        printf("failed to assert remote participant(localization_dp)\n");
        goto done;
    }
    retcode = DPSE_RemoteParticipant_assert(
        application->participant, "egoMotion_dp");     /* Note: this string should match the remote participant name */
    if (retcode != DDS_RETCODE_OK)  {
        printf("failed to assert remote participant(egoMotion_dp)\n");
        goto done;
    }
    retcode = DPSE_RemoteParticipant_assert(
        application->participant, "sceneEval_dp");     /* Note: this string should match the remote participant name */
    if (retcode != DDS_RETCODE_OK)  {
        printf("failed to assert remote participant(sceneEval_dp)\n");
        goto done;
    }
    retcode = DPSE_RemoteParticipant_assert(
        application->participant, "stimAndPrint_dp");     /* Note: this string should match the remote participant name */
    if (retcode != DDS_RETCODE_OK)  {
        printf("failed to assert remote participant(stimAndPrint_dp)\n");
        goto done;
    }

    // create publisher and dw_qos --------------
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
	ObjectsDetected_dw_qos.protocol.rtps_object_id = 100;  /* Note: must assign unique numbers to each instance */

    /* create datawriters for each pub topic */
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

    ObjectsDetected_rem_subscription_data.topic_name = DDS_String_dup("detectedObjects");
    ObjectsDetected_rem_subscription_data.type_name = DDS_String_dup(dds_perception_ObjectsDetectedTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    ObjectsDetected_rem_subscription_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    ObjectsDetected_rem_subscription_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    /* assert for every subscriber of this topic data */
    ObjectsDetected_rem_subscription_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 210;
    if (DDS_RETCODE_OK != DPSE_RemoteSubscription_assert(
        application->participant, "localization_dp",   /* Note: this string should match the remote participant name */
        &ObjectsDetected_rem_subscription_data,
        dds_perception_ObjectsDetected_get_key_kind(dds_perception_ObjectsDetectedTypePlugin_get(), NULL)))
    {
        printf("failed to assert remote subscription(localization_dp::ObjectsDetected)\n");
        goto done;
    }
    ObjectsDetected_rem_subscription_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 220;
    if (DDS_RETCODE_OK != DPSE_RemoteSubscription_assert(
        application->participant, "egoMotion_dp",   /* Note: this string should match the remote participant name */
        &ObjectsDetected_rem_subscription_data,
        dds_perception_ObjectsDetected_get_key_kind(dds_perception_ObjectsDetectedTypePlugin_get(), NULL)))
    {
        printf("failed to assert remote subscription(egoMotion_dp::ObjectsDetected)\n");
        goto done;
    }
    ObjectsDetected_rem_subscription_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 230;
    if (DDS_RETCODE_OK != DPSE_RemoteSubscription_assert(
        application->participant, "sceneEval_dp",   /* Note: this string should match the remote participant name */
        &ObjectsDetected_rem_subscription_data,
        dds_perception_ObjectsDetected_get_key_kind(dds_perception_ObjectsDetectedTypePlugin_get(), NULL)))
    {
        printf("failed to assert remote subscription(sceneEval_dp::ObjectsDetected)\n");
        goto done;
    }

    ObjectsDetected_hw_datawriter = dds_perception_ObjectsDetectedDataWriter_narrow(ObjectsDetected_datawriter);
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
#ifdef USE_SAMPLE_FILTER
    Lidar_dr_qos.resource_limits.max_instances = 1;
#else  /* USE_SAMPLE_FILTER */
    Lidar_dr_qos.resource_limits.max_instances = 2;
#endif  /* USE_SAMPLE_FILTER */

    Lidar_dr_qos.resource_limits.max_samples_per_instance = 32;
    Lidar_dr_qos.resource_limits.max_samples = Lidar_dr_qos.resource_limits.max_instances *
        Lidar_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    Lidar_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Lidar_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Lidar_dr_qos.history.depth = 32;

    /* Publisher sends samples with id = 0 or id = 1, so 2 instances maximum.
    * But in case filtering is done, all samples with 'id = 0' are
    * filtered so only one instance is needed.
    */
#ifdef USE_SAMPLE_FILTER
    Camera_dr_qos.resource_limits.max_instances = 1;
#else  /* USE_SAMPLE_FILTER */
    Camera_dr_qos.resource_limits.max_instances = 2;
#endif  /* USE_SAMPLE_FILTER */

    Camera_dr_qos.resource_limits.max_samples_per_instance = 32;
    Camera_dr_qos.resource_limits.max_samples = Camera_dr_qos.resource_limits.max_instances *
        Camera_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    Camera_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Camera_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Camera_dr_qos.history.depth = 32;

    /* Publisher sends samples with id = 0 or id = 1, so 2 instances maximum.
    * But in case filtering is done, all samples with 'id = 0' are
    * filtered so only one instance is needed.
    */
#ifdef USE_SAMPLE_FILTER
    Pose3D_real_dr_qos.resource_limits.max_instances = 1;
#else  /* USE_SAMPLE_FILTER */
    Pose3D_real_dr_qos.resource_limits.max_instances = 2;
#endif  /* USE_SAMPLE_FILTER */

    Pose3D_real_dr_qos.resource_limits.max_samples_per_instance = 32;
    Pose3D_real_dr_qos.resource_limits.max_samples = Pose3D_real_dr_qos.resource_limits.max_instances *
        Pose3D_real_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    Pose3D_real_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Pose3D_real_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Pose3D_real_dr_qos.history.depth = 32;

    /* Publisher sends samples with id = 0 or id = 1, so 2 instances maximum.
    * But in case filtering is done, all samples with 'id = 0' are
    * filtered so only one instance is needed.
    */
#ifdef USE_SAMPLE_FILTER
    Trajectory_dr_qos.resource_limits.max_instances = 1;
#else  /* USE_SAMPLE_FILTER */
    Trajectory_dr_qos.resource_limits.max_instances = 2;
#endif  /* USE_SAMPLE_FILTER */

    Trajectory_dr_qos.resource_limits.max_samples_per_instance = 32;
    Trajectory_dr_qos.resource_limits.max_samples = Trajectory_dr_qos.resource_limits.max_instances *
        Trajectory_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    Trajectory_dr_qos.reader_resource_limits.max_remote_writers = 10;
    Trajectory_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    Trajectory_dr_qos.history.depth = 32;

    /* Publisher sends samples with id = 0 or id = 1, so 2 instances maximum.
    * But in case filtering is done, all samples with 'id = 0' are
    * filtered so only one instance is needed.
    */
#ifdef USE_SAMPLE_FILTER
    MapData_dr_qos.resource_limits.max_instances = 1;
#else  /* USE_SAMPLE_FILTER */
    MapData_dr_qos.resource_limits.max_instances = 2;
#endif  /* USE_SAMPLE_FILTER */

    MapData_dr_qos.resource_limits.max_samples_per_instance = 32;
    MapData_dr_qos.resource_limits.max_samples = MapData_dr_qos.resource_limits.max_instances *
        MapData_dr_qos.resource_limits.max_samples_per_instance;
    /* if there are more remote writers, you need to increase these limits */
    MapData_dr_qos.reader_resource_limits.max_remote_writers = 10;
    MapData_dr_qos.reader_resource_limits.max_remote_writers_per_instance = 10;
    MapData_dr_qos.history.depth = 32;

#ifdef USE_SAMPLE_FILTER
    /* choose one callback to enable */
#ifdef FILTER_ON_DESERIALIZE
    Lidar_dr_listener.on_before_sample_deserialize =
        dds_sensing_LidarSubscriber_on_before_sample_deserialize;
#else  /* FILTER_ON_DESERIALIZE */
    Lidar_dr_listener.on_before_sample_commit =
        dds_sensing_LidarSubscriber_on_before_sample_commit;
#endif  /* FILTER_ON_DESERIALIZE */
#endif  /* USE_SAMPLE_FILTER */

    Lidar_dr_listener.on_data_available = dds_sensing_LidarSubscriber_on_data_available;
    Lidar_dr_listener.on_subscription_matched =
        dds_sensing_LidarSubscriber_on_subscription_matched;

    Lidar_dr_listener.as_listener.listener_data = &total_samples;
    Lidar_dr_qos.protocol.rtps_object_id = 201;  /* Note: must assign unique numbers to each instance */

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

    Lidar_rem_publication_data.topic_name = DDS_String_dup("lidarPoints");
    Lidar_rem_publication_data.type_name = DDS_String_dup(dds_sensing_LidarTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    Lidar_rem_publication_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    Lidar_rem_publication_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    Lidar_rem_publication_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 141;
    retcode = DPSE_RemotePublication_assert(
        application->participant, "stimAndPrint_dp",  /* Note: this string should match the remote participant name */
        &Lidar_rem_publication_data,
        dds_sensing_Lidar_get_key_kind(dds_sensing_LidarTypePlugin_get(), NULL));
    if (retcode != DDS_RETCODE_OK) {
        printf("failed to assert remote publication for stimAndPrint_dp::Lidar(%d)\n", __LINE__);
        goto done;
    }

#ifdef USE_SAMPLE_FILTER
    /* choose one callback to enable */
#ifdef FILTER_ON_DESERIALIZE
    Camera_dr_listener.on_before_sample_deserialize =
        dds_sensing_CameraSubscriber_on_before_sample_deserialize;
#else  /* FILTER_ON_DESERIALIZE */
    Camera_dr_listener.on_before_sample_commit =
        dds_sensing_CameraSubscriber_on_before_sample_commit;
#endif  /* FILTER_ON_DESERIALIZE */
#endif  /* USE_SAMPLE_FILTER */

    Camera_dr_listener.on_data_available = dds_sensing_CameraSubscriber_on_data_available;
    Camera_dr_listener.on_subscription_matched =
        dds_sensing_CameraSubscriber_on_subscription_matched;

    Camera_dr_listener.as_listener.listener_data = &total_samples;
    Camera_dr_qos.protocol.rtps_object_id = 202;  /* Note: must assign unique numbers to each instance */

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

    Camera_rem_publication_data.topic_name = DDS_String_dup("cameraView");
    Camera_rem_publication_data.type_name = DDS_String_dup(dds_sensing_CameraTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    Camera_rem_publication_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    Camera_rem_publication_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    Camera_rem_publication_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 142;
    retcode = DPSE_RemotePublication_assert(
        application->participant,
        "stimAndPrint_dp",           /* Note: this string should match the remote participant name */
        &Camera_rem_publication_data,
        dds_sensing_Camera_get_key_kind(dds_sensing_CameraTypePlugin_get(), NULL));

    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to assert remote publication(%d)\n", __LINE__);
        goto done;
    }

#ifdef USE_SAMPLE_FILTER
    /* choose one callback to enable */
#ifdef FILTER_ON_DESERIALIZE
    Pose3D_real_dr_listener.on_before_sample_deserialize =
        dds_physics_Pose3D_realSubscriber_on_before_sample_deserialize;
#else  /* FILTER_ON_DESERIALIZE */
    Pose3D_real_dr_listener.on_before_sample_commit =
        dds_physics_Pose3D_realSubscriber_on_before_sample_commit;
#endif  /* FILTER_ON_DESERIALIZE */
#endif  /* USE_SAMPLE_FILTER */

    Pose3D_real_dr_listener.on_data_available = dds_physics_Pose3D_realSubscriber_on_data_available;
    Pose3D_real_dr_listener.on_subscription_matched =
        dds_physics_Pose3D_realSubscriber_on_subscription_matched;

    Pose3D_real_dr_listener.as_listener.listener_data = &total_samples;
    Pose3D_real_dr_qos.protocol.rtps_object_id = 203;  /* Note: must assign unique numbers to each instance */

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

    Pose3D_real_rem_publication_data.topic_name = DDS_String_dup("egoPose");
    Pose3D_real_rem_publication_data.type_name = DDS_String_dup(dds_physics_Pose3D_realTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    Pose3D_real_rem_publication_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    Pose3D_real_rem_publication_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    Pose3D_real_rem_publication_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 113;
    retcode = DPSE_RemotePublication_assert(
        application->participant,
        "localization_dp",           /* Note: this string should match the remote participant name */
        &Pose3D_real_rem_publication_data,
        dds_physics_Pose3D_real_get_key_kind(dds_physics_Pose3D_realTypePlugin_get(), NULL));

    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to assert remote publication(%d)\n", __LINE__);
        goto done;
    }

#ifdef USE_SAMPLE_FILTER
    /* choose one callback to enable */
#ifdef FILTER_ON_DESERIALIZE
    Trajectory_dr_listener.on_before_sample_deserialize =
        dds_perception_TrajectorySubscriber_on_before_sample_deserialize;
#else  /* FILTER_ON_DESERIALIZE */
    Trajectory_dr_listener.on_before_sample_commit =
        dds_perception_TrajectorySubscriber_on_before_sample_commit;
#endif  /* FILTER_ON_DESERIALIZE */
#endif  /* USE_SAMPLE_FILTER */

    Trajectory_dr_listener.on_data_available = dds_perception_TrajectorySubscriber_on_data_available;
    Trajectory_dr_listener.on_subscription_matched =
        dds_perception_TrajectorySubscriber_on_subscription_matched;

    Trajectory_dr_listener.as_listener.listener_data = &total_samples;
    Trajectory_dr_qos.protocol.rtps_object_id = 204;  /* Note: must assign unique numbers to each instance */

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

    Trajectory_rem_publication_data.topic_name = DDS_String_dup("egoMotion");
    Trajectory_rem_publication_data.type_name = DDS_String_dup(dds_perception_TrajectoryTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    Trajectory_rem_publication_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    Trajectory_rem_publication_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    Trajectory_rem_publication_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 124;
    retcode = DPSE_RemotePublication_assert(
        application->participant,
        "egoMotion_dp",           /* Note: this string should match the remote participant name */
        &Trajectory_rem_publication_data,
        dds_perception_Trajectory_get_key_kind(dds_perception_TrajectoryTypePlugin_get(), NULL));

    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to assert remote publication(%d)\n", __LINE__);
        goto done;
    }

#ifdef USE_SAMPLE_FILTER
    /* choose one callback to enable */
#ifdef FILTER_ON_DESERIALIZE
    MapData_dr_listener.on_before_sample_deserialize =
        dds_planning_MapDataSubscriber_on_before_sample_deserialize;
#else  /* FILTER_ON_DESERIALIZE */
    MapData_dr_listener.on_before_sample_commit =
        dds_planning_MapDataSubscriber_on_before_sample_commit;
#endif  /* FILTER_ON_DESERIALIZE */
#endif  /* USE_SAMPLE_FILTER */

    MapData_dr_listener.on_data_available = dds_planning_MapDataSubscriber_on_data_available;
    MapData_dr_listener.on_subscription_matched =
        dds_planning_MapDataSubscriber_on_subscription_matched;

    MapData_dr_listener.as_listener.listener_data = &total_samples;
    MapData_dr_qos.protocol.rtps_object_id = 205;  /* Note: must assign unique numbers to each instance */

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


    MapData_rem_publication_data.topic_name = DDS_String_dup("mapData");
    MapData_rem_publication_data.type_name = DDS_String_dup(dds_planning_MapDataTypeSupport_get_type_name());

#ifdef USE_RELIABLE_QOS
    MapData_rem_publication_data.reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;
#else
    MapData_rem_publication_data.reliability.kind  = DDS_BEST_EFFORT_RELIABILITY_QOS;
#endif

    MapData_rem_publication_data.key.value[DDS_BUILTIN_TOPIC_KEY_OBJECT_ID] = 145;
    retcode = DPSE_RemotePublication_assert(
        application->participant,
        "stimAndPrint_dp",           /* Note: this string should match the remote participant name */
        &MapData_rem_publication_data,
        dds_planning_MapData_get_key_kind(dds_planning_MapDataTypePlugin_get(), NULL));

    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to assert remote publication(%d)\n", __LINE__);
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
        /* TODO set sample attributes here */
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
            printf("Written ObjectsDetected sample %d\n",(i+1));
        }

        printf(" Subscriber sleeping for %d msec...\n", sleep_time);

        /* OSAPI_Thread_sleep() is not included in Connext DDS Cert because it might not
        * be thread safe. For this reason a call to OSAPI_Thread_sleep() cannot be done
        * in this example. Instead we use the code below.
        */
#if OSAPI_INCLUDE_POSIX
        {
            struct timespec remain, next;
            int rval;
            RTI_UINT32 is;

            is = application->sleep_time / 1000;

            next.tv_sec = is;
            next.tv_nsec = (application->sleep_time - (is * 1000U)) * 1000000U;

            do
            {
                rval = nanosleep(&next, &remain);
                if ((rval == -1) && (errno == EINTR))
                {
                    next = remain;
                }
            } while ((rval == -1) && (errno == EINTR));
        }
#elif RTI_VXWORKS
        taskDelay(((sysClkRateGet() * application->sleep_time) + 500) / 1000);
#else
        #error "Implementation of sleep() needed"
#endif
    }

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
    if (DDS_SubscriptionBuiltinTopicData_finalize(&ObjectsDetected_rem_subscription_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for ObjectsDetected\n");
        return -1;
    }

    retcode = DDS_DataReaderQos_finalize(&Lidar_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    if (DDS_PublicationBuiltinTopicData_finalize(&Lidar_rem_publication_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for Lidar\n");
        return -1;
    }

    retcode = DDS_DataReaderQos_finalize(&Camera_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    if (DDS_PublicationBuiltinTopicData_finalize(&Camera_rem_publication_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for Camera\n");
        return -1;
    }

    retcode = DDS_DataReaderQos_finalize(&Pose3D_real_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    if (DDS_PublicationBuiltinTopicData_finalize(&Pose3D_real_rem_publication_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for Pose3D_real\n");
        return -1;
    }

    retcode = DDS_DataReaderQos_finalize(&Trajectory_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    if (DDS_PublicationBuiltinTopicData_finalize(&Trajectory_rem_publication_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for Trajectory\n");
        return -1;
    }

    retcode = DDS_DataReaderQos_finalize(&MapData_dr_qos);
    if (retcode != DDS_RETCODE_OK)
    {
        printf("Cannot finalize DataReaderQos\n");
        return -1;
    }
    if (DDS_PublicationBuiltinTopicData_finalize(&MapData_rem_publication_data) !=
    DDS_BOOLEAN_TRUE)
    {
        printf("Cannot finalize DDS_SubscriptionBuiltinTopicData for MapData\n");
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
