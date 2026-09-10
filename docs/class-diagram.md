# Ouster ROS - Inheritance and Aggregation Chart

## Class Hierarchy Overview

```mermaid
classDiagram
    %% ROS2 Base Classes
    class rclcpp_lifecycle_LifecycleNode {
        <<interface>>
    }

    class rclcpp_Node {
        <<interface>>
    }

    %% Base Classes
    class OusterSensorNodeBase {
        +sensor_info info
        +SensorDiagnosticsTracker diagnostics_tracker
        +rclcpp::Publisher metadata_pub
        +rclcpp::Service get_metadata_srv
        +create_get_metadata_service()
        +create_metadata_pub()
        +publish_metadata()
        +create_diagnostics_pub()
        +publish_diagnostics()
        +update_diagnostics_status()
    }

    class OusterProcessingNodeBase {
        +sensor_info info
        +rclcpp::Subscription metadata_sub
        +create_metadata_subscriber()
    }

    %% Core Node Classes
    class OusterSensor {
        -sensor_client: shared_ptr<client>
        -lidar_packet_pub: Publisher<PacketMsg>
        -imu_packet_pub: Publisher<PacketMsg>
        -sensor_connection_thread: unique_ptr<thread>
        -staged_config: optional<sensor_config>
        +start()
        +on_configure()
        +on_activate()
        +reset_sensor()
        +create_sensor_client()
        +connection_loop()
    }

    class OusterDriver {
        -tf_bcast: OusterStaticTransformsBroadcaster
        -imu_handler: shared_ptr<ImuPacketHandler>
        -lidar_handler: shared_ptr<LidarPacketHandler>
        -image_handler: shared_ptr<ImageProcessor>
        -telemetry_handler: shared_ptr<TelemetryHandler>
        +on_metadata_updated()
        +create_services()
        +create_publishers()
    }

    class OusterCloud {
        -tf_bcast: OusterStaticTransformsBroadcaster
        -imu_handler: shared_ptr<ImuPacketHandler>
        -lidar_handler: shared_ptr<LidarPacketHandler>
        -image_handler: shared_ptr<ImageProcessor>
        +metadata_handler()
        +create_publishers_subscriptions()
    }

    class OusterImage {
        -image_handler: shared_ptr<ImageProcessor>
        +metadata_handler()
        +create_publishers_subscriptions()
    }

    class OusterReplay {
        -rosbag_reader: shared_ptr<BagReader>
        -replay_timer: rclcpp::Timer
        +load_bag_file()
        +replay_messages()
    }

    class OusterPcap {
        -pcap_reader: unique_ptr<PcapReader>
        -replay_timer: rclcpp::Timer
        +load_pcap_file()
        +process_packets()
    }

    %% Processing Classes
    class LidarPacketHandler {
        -scan_batcher: unique_ptr<ScanBatcher>
        -lidar_scans: vector<unique_ptr<LidarScan>>
        -ring_buffer: LockFreeRingBuffer
        -lidar_scan_handlers: vector<LidarScanProcessor>
        +register_lidar_scan_handler()
        +process_scans()
        +lidar_handler_sensor_time()
        +lidar_handler_ros_time()
    }

    class PointCloudProcessor~PointT~ {
        -cloud: Cloud<PointT>
        -pc_msgs: vector<shared_ptr<PointCloud2>>
        -scan_to_cloud_fn: ScanToCloudFn
        -post_processing_fn: PostProcessingFn
        +process()
        +create()
    }

    class ImuPacketHandler {
        -imu_pub: Publisher<Imu>
        +process_packet()
        +create()
    }

    class ImageProcessor {
        -image_pubs: vector<Publisher<Image>>
        +process()
        +create()
    }

    class LaserScanProcessor {
        -scan_pub: Publisher<LaserScan>
        +process()
        +create()
    }

    class TelemetryHandler {
        -telemetry_pub: Publisher<String>
        +process()
        +create()
    }

    %% Utility Classes
    class SensorDiagnosticsTracker {
        -name_: string
        -hardware_id_: string
        -clock_: shared_ptr<Clock>
        -sensor_start_time_: Time
        +record_lidar_packet()
        +record_imu_packet()
        +create_diagnostic_status()
        +update_metadata()
    }

    class OusterStaticTransformsBroadcaster {
        -tf_broadcaster: shared_ptr<StaticTransformBroadcaster>
        +broadcast_transforms()
        +declare_parameters()
        +parse_parameters()
    }

    class LockFreeRingBuffer {
        -buffer_size: size_t
        -read_head_: atomic<size_t>
        -write_head_: atomic<size_t>
        +write()
        +read()
        +empty()
        +full()
    }

    class PointCloudProcessorFactory {
        +create_processor()
    }

    %% Inheritance Relationships
    rclcpp_lifecycle_LifecycleNode <|-- OusterSensorNodeBase
    rclcpp_Node <|-- OusterProcessingNodeBase

    OusterSensorNodeBase <|-- OusterSensor
    OusterSensorNodeBase <|-- OusterReplay
    OusterSensorNodeBase <|-- OusterPcap

    OusterSensor <|-- OusterDriver

    OusterProcessingNodeBase <|-- OusterCloud
    OusterProcessingNodeBase <|-- OusterImage

    %% Aggregation Relationships (Composition/Has-a)
    OusterSensorNodeBase *-- SensorDiagnosticsTracker : contains

    OusterDriver *-- OusterStaticTransformsBroadcaster : contains
    OusterDriver *-- ImuPacketHandler : contains
    OusterDriver *-- LidarPacketHandler : contains
    OusterDriver *-- ImageProcessor : contains
    OusterDriver *-- TelemetryHandler : contains

    OusterCloud *-- OusterStaticTransformsBroadcaster : contains
    OusterCloud *-- ImuPacketHandler : contains
    OusterCloud *-- LidarPacketHandler : contains
    OusterCloud *-- ImageProcessor : contains

    OusterImage *-- ImageProcessor : contains

    LidarPacketHandler *-- LockFreeRingBuffer : contains
    LidarPacketHandler o-- PointCloudProcessor : uses
    LidarPacketHandler o-- LaserScanProcessor : uses

    PointCloudProcessor *-- PointCloudProcessorFactory : creates via
```

## Key Architectural Patterns

### 1. **Inheritance Hierarchy**

#### Base Classes:
- **OusterSensorNodeBase**: Lifecycle-managed sensor nodes with diagnostics
- **OusterProcessingNodeBase**: Simple processing nodes that subscribe to metadata

#### Concrete Node Classes:
- **OusterSensor**: Basic sensor interface (publishes raw packets)
- **OusterDriver**: Full-featured sensor node (inherits from OusterSensor)
- **OusterCloud**: Point cloud processing node
- **OusterImage**: Image processing node
- **OusterReplay**: Bag file replay node
- **OusterPcap**: PCAP file replay node

### 2. **Composition and Aggregation**

#### Core Processing Components:
- **LidarPacketHandler**: Handles packet batching and threading
- **ImuPacketHandler**: Processes IMU data
- **ImageProcessor**: Converts lidar data to images
- **PointCloudProcessor<T>**: Template-based point cloud generation
- **LaserScanProcessor**: Generates 2D laser scans
- **TelemetryHandler**: Manages sensor telemetry

#### Utility Components:
- **SensorDiagnosticsTracker**: Monitors sensor health
- **OusterStaticTransformsBroadcaster**: Publishes TF transforms
- **LockFreeRingBuffer**: Thread-safe circular buffer
- **PointCloudProcessorFactory**: Factory pattern for processors

### 3. **Design Patterns Used**

1. **Template Pattern**: `PointCloudProcessor<PointT>` for different point types
2. **Factory Pattern**: `PointCloudProcessorFactory` for processor creation
3. **Handler/Strategy Pattern**: Different packet processors
4. **Observer Pattern**: Multiple handlers for lidar scan processing
5. **RAII Pattern**: Automatic resource management in handlers
6. **Producer-Consumer Pattern**: `LockFreeRingBuffer` for threading

### 4. **Threading Architecture**

- **OusterSensor**: Dedicated connection thread for sensor communication
- **LidarPacketHandler**: Processing thread with lock-free ring buffer
- **Lock-free design**: Minimizes blocking between producer and consumer threads

### 5. **Key Relationships**

#### Strong Composition (ownership):
- Nodes own their processors and handlers
- Processors own their internal data structures

#### Weak Aggregation (usage):
- Handlers use multiple processors via callbacks
- Factory creates processors but doesn't own them
- Processors can be shared among handlers

This architecture provides a flexible, modular design that separates concerns while maintaining efficient data flow through the processing pipeline.
