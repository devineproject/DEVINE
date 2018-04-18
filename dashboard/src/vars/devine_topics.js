export const devineTopics = {
    object_found: {
        name: "/object_found",
        type: "std_msgs/Int32MultiArray"
    },
    object_location: {
        name: "/object_location",
        type: "std_msgs/Int32MultiArray"
    },
    guesswhat_state: {
        name: "/guesswhat_state",
        type: "std_msgs/String"
    },
    found_category: {
        name: "/found_category",
        type: "std_msgs/String"
    },
    segmentation: {
        name: "/rcnn_segmentation",
        type: "std_msgs/String"
    },
    segmentation_image: {
        name: "/devine/image",
        type: "sensor_msgs/CompressedImage"
    },
    image: {
        name: "/camera/rgb/image_color/compressed",
        type: "sensor_msgs/CompressedImage"
    },
    answer: {
        name: "/answer",
        type: "std_msgs/String"
    },
    question: {
        name: "/question",
        type: "std_msgs/String"
    }
}