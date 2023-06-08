# image_transport_wrapper

This repo allows to work with image_transport. It provides examples to publish and subscribe images and a wrapper utility, which subscribes to an image topic and publishes the image using image_transport with all plugins available

Publish an image using all the transport plugins from image_transport:
```
ros2 run image_transport_wrapper image_publisher <path-to-file>
```
Subscribe to a topic using the transport plugin selected:
```
ros2 run image_transport_wrapper image_subscriber --ros-args -p topic_image:=/camera/image_raw -p transport:=compressed 
```
Subscribe to a raw topic and publish the image using all the transport plugins from image_transport:
```
ros2 run image_transport_wrapper wrapper_publisher --ros-args -p topic_in:=/camera/image_raw -p topic_out:=/out/image
```
Open rqt_image_view:
```
ros2 run rqt_image_view rqt_image_view --force-discover
```
