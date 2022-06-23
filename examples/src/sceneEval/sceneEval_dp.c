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
/* sceneEval_dp.c
* Helper file: create and configure the domain participant
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rti_me_c.h"
#include "disc_dpde/disc_dpde_discovery_plugin.h"
#include "wh_sm/wh_sm_history.h"
#include "rh_sm/rh_sm_history.h"
#include "netio/netio_udp.h"
#include "sceneEval_dp.h"

/* app command line help.  Called from main() */
void
Application_help(char *appname)
{
    printf("%s [options]\n", appname);
    printf("options:\n");
    printf("-h                      - This text\n");
    printf("-domain <id>            - DomainId (default: 0)\n");
    printf("-udp_intf <intf>        - udp interface (no default)\n");
    printf("-peer <address>         - peer address (no default)\n");
    printf("-count <count>          - count (default 0. Runs forever.)\n");
    printf("-sleep <ms>             - sleep between sends (default 1s)\n");
    printf("\n");
}

/* Application_create()
   Called once to set up the domain participant for the application */
struct Application *
Application_create(
    DDS_Long domain_id,
    char *udp_intf,
    char *peer)
{
    DDS_ReturnCode_t retcode;
    DDS_DomainParticipantFactory *factory = NULL;
    struct DDS_DomainParticipantQos dp_qos =
    DDS_DomainParticipantQos_INITIALIZER;
    DDS_Boolean success = DDS_BOOLEAN_FALSE;
    RT_Registry_T *registry = NULL;
    struct UDP_InterfaceFactoryProperty *udp_property = NULL;

    struct DPDE_DiscoveryPluginProperty discovery_plugin_properties =
        DPDE_DiscoveryPluginProperty_INITIALIZER;

    struct Application *application = NULL;
    /* Uncomment to increase verbosity level:
    OSAPI_Log_set_verbosity(OSAPI_LOG_VERBOSITY_WARNING);
    */
    application = (struct Application *)malloc(sizeof(struct Application));

    if (application == NULL)
    {
        printf("failed to allocate application\n");
        goto done;
    }

    factory = DDS_DomainParticipantFactory_get_instance();
    registry = DDS_DomainParticipantFactory_get_registry(factory);

    if (!RT_Registry_register(
        registry,
        DDSHST_WRITER_DEFAULT_HISTORY_NAME,
        WHSM_HistoryFactory_get_interface(),
        NULL,
        NULL))
    {
        printf("failed to register wh\n");
        goto done;
    }

    if (!RT_Registry_register(
        registry,
        DDSHST_READER_DEFAULT_HISTORY_NAME,
        RHSM_HistoryFactory_get_interface(),
        NULL,
        NULL))
    {
        printf("failed to register rh\n");
        goto done;
    }

    /* Configure UDP transport's allowed interfaces */
    if (!RT_Registry_unregister(registry, NETIO_DEFAULT_UDP_NAME, NULL, NULL))
    {
        printf("failed to unregister udp\n");
        goto done;
    }

    udp_property = (struct UDP_InterfaceFactoryProperty *)
    malloc(sizeof(struct UDP_InterfaceFactoryProperty));
    if (udp_property == NULL)
    {
        printf("failed to allocate udp properties\n");
        goto done;
    }
    *udp_property = UDP_INTERFACE_FACTORY_PROPERTY_DEFAULT;

    /* To add more allowed interface(s), increase maximum and length,
       and set interface below:
    */
    if (!DDS_StringSeq_set_maximum(&udp_property->allow_interface,2))
    {
        printf("failed to set allow_interface maximum\n");
        goto done;
    }
    if (!DDS_StringSeq_set_length(&udp_property->allow_interface,2))
    {
        printf("failed to set allow_interface length\n");
        goto done;
    }

    /* loopback interface */
    #if defined(RTI_DARWIN)
    *DDS_StringSeq_get_reference(&udp_property->allow_interface,0) =
    DDS_String_dup("lo0");
    #elif defined (RTI_LINUX)
    *DDS_StringSeq_get_reference(&udp_property->allow_interface,0) =
    DDS_String_dup("lo");
    #elif defined (RTI_VXWORKS)
    *DDS_StringSeq_get_reference(&udp_property->allow_interface,0) =
    DDS_String_dup("lo0");
    #elif defined(RTI_WIN32)
    *DDS_StringSeq_get_reference(&udp_property->allow_interface,0) =
    DDS_String_dup("Loopback Pseudo-Interface 1");
    #else
    *DDS_StringSeq_get_reference(&udp_property->allow_interface,0) =
    DDS_String_dup("lo");
    #endif

    if (udp_intf != NULL)
    { /* use interface supplied on command line */
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup(udp_intf);
    }
    else                /* use hardcoded interface */
    {
        #if defined(RTI_DARWIN)
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup("en1");
        #elif defined (RTI_LINUX)
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup("eth0");
        #elif defined (RTI_VXWORKS)
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup("geisc0");
        #elif defined(RTI_WIN32)
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup("Local Area Connection");
        #else
        *DDS_StringSeq_get_reference(&udp_property->allow_interface,1) =
        DDS_String_dup("ce0");
        #endif
    }

    if (!RT_Registry_register(
        registry,
        NETIO_DEFAULT_UDP_NAME,
        UDP_InterfaceFactory_get_interface(),
        (struct RT_ComponentFactoryProperty*)udp_property,
        NULL))
    {
        printf("failed to register udp\n");
        goto done;
    }

    if (peer == NULL)
    {
        peer = "127.0.0.1"; /* default to loopback */
    }

    if (!RT_Registry_register(
        registry,
        "dpde",
        DPDE_DiscoveryFactory_get_interface(),
        &discovery_plugin_properties._parent,
        NULL))
    {
        printf("failed to register dpde\n");
        goto done;
    }

    if (!RT_ComponentFactoryId_set_name(&dp_qos.discovery.discovery.name,"dpde"))
    {
        printf("failed to set discovery plugin name\n");
        goto done;
    }

    if (!DDS_StringSeq_set_maximum(&dp_qos.discovery.initial_peers,1))
    {
        printf("failed to set initial peers maximum\n");
        goto done;
    }
    if (!DDS_StringSeq_set_length(&dp_qos.discovery.initial_peers,1))
    {
        printf("failed to set initial peers length\n");
        goto done;
    }
    *DDS_StringSeq_get_reference(&dp_qos.discovery.initial_peers,0) = DDS_String_dup(peer);

    /* The resource limits for the Domain Participant are set here;
       If local or remote endpoints are added or removed, you may need
       to adjust these values */
    dp_qos.resource_limits.max_destination_ports = 32;
    dp_qos.resource_limits.max_receive_ports = 32;
    dp_qos.resource_limits.local_topic_allocation = 5;
    dp_qos.resource_limits.local_type_allocation = 5;
    dp_qos.resource_limits.local_reader_allocation = 4;
    dp_qos.resource_limits.local_writer_allocation = 1;
    dp_qos.resource_limits.remote_participant_allocation = 8;
    dp_qos.resource_limits.remote_reader_allocation = 8;
    dp_qos.resource_limits.remote_writer_allocation = 8;

    application->participant = DDS_DomainParticipantFactory_create_participant(
        factory,
        domain_id,
        &dp_qos,
        NULL,
        DDS_STATUS_MASK_NONE);
    if (application->participant == NULL)
    {
        printf("failed to create participant\n");
        goto done;
    }


    success = DDS_BOOLEAN_TRUE;

    done:
    #ifndef RTI_CERT
    DDS_DomainParticipantQos_finalize(&dp_qos);
    #endif

    if (!success)
    {
        #ifndef RTI_CERT
        if (udp_property != NULL)
        {
            UDP_InterfaceFactoryProperty_finalize(udp_property);
            free(udp_property);
        }
        #endif

        if (application != NULL)
        {
            #ifndef RTI_CERT
            free(application);
            #endif
            application = NULL;
        }
    }

    return application;
}

#ifndef RTI_CERT
void
Application_delete(struct Application *application)
{
    DDS_ReturnCode_t retcode;
    RT_Registry_T *registry = NULL;
    DDS_DomainParticipantFactory *factory = NULL;
    struct UDP_InterfaceFactoryProperty *udp_property = NULL;

    if (application == NULL)
    {
        return;
    }

    factory = DDS_DomainParticipantFactory_get_instance();

    if (application->participant != NULL)
    {
        retcode = DDS_DomainParticipant_delete_contained_entities(
            application->participant);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("failed to delete contained entities (retcode=%d)\n",retcode);
        }

        retcode = DDS_DomainParticipantFactory_delete_participant(
            factory,
            application->participant);
        if (retcode != DDS_RETCODE_OK)
        {
            printf("failed to delete participant: %d\n", retcode);
            return;
        }
    }

    registry = DDS_DomainParticipantFactory_get_registry(factory);

    if (!RT_Registry_unregister(
        registry,
        NETIO_DEFAULT_UDP_NAME,
        (struct RT_ComponentFactoryProperty**)&udp_property,
        NULL))
    {
        printf("failed to unregister udp\n");
        return;
    }
    if (udp_property != NULL)
    {
        UDP_InterfaceFactoryProperty_finalize(udp_property);
        free(udp_property);
        udp_property = NULL;
    }

    if (!RT_Registry_unregister(registry, "dpde", NULL, NULL))
    {
        printf("failed to unregister dpde\n");
        return;
    }
    if (!RT_Registry_unregister(
        registry,
        DDSHST_READER_DEFAULT_HISTORY_NAME,
        NULL,
        NULL))
    {
        printf("failed to unregister rh\n");
        return;
    }

    if (!RT_Registry_unregister(
        registry,
        DDSHST_WRITER_DEFAULT_HISTORY_NAME,
        NULL,
        NULL))
    {
        printf("failed to unregister wh\n");
        return;
    }

    free(application);

    retcode = DDS_DomainParticipantFactory_finalize_instance();
    if (retcode != DDS_RETCODE_OK)
    {
        printf("failed to finalize instance %d\n", retcode);
        return;
    }
}

#endif /* !RTI_CERT */
