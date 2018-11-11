Image pipeline
#####################

Being able to interface with GuessWhat?! and users requires taking inputs from the robot's Kinect 360 and processing them accordingly. The first link in the chain is the image dispatcher, which takes compressed images, validates that they are not blurred, and based on the game state, sends them onto the next node in the chain.

The next node to recieve the image, temporally, is the body tracking node. Using OpenPose we try to determine if a person is within range to begin a game. If it is the case, after the scene's picture is taken, the image is sent to the segmentation and feature extration nodes.

Interfacing with GuessWhat?! requires extracting: a list of all objects, bounding boxes around them and a feature vector (FC8 of a VGG16). Respectively the segmentatation and feature nodes are responsible for this.

Below is a UML showing the sequence of interactions between the different modules.


.. uml::

    @startuml
    'Look params'
    skinparam sequence {
        hide footbox
        shadowing false
        ParticipantPadding 20
        BoxPadding 10
        ArrowColor Black
        ActorBorderColor Black
        LifeLineBorderColor Black
        LifeLineBackgroundColor Black
        
        ParticipantBorderColor Black
        ParticipantBackgroundColor Grey
        ParticipantFontName Roboto
        ParticipantFontSize 18
        ParticipantFontColor White
        
        ActorBackgroundColor White
        ActorFontColor Black
        ActorFontSize 18
        ActorFontName Roboto
        
        BoxFontSize 16
        TitleFontSize 20
     }
    'Box and participants'
    box "Robot CPU" #FFFFFF
        participant openni
        participant body_tracking
        participant image_dispatcher
        participant feature_extraction
        participant image_segmentation
        participant scene_finder
        participant GuessWhat
    end box

     'UML'
    title Start Game

     == Wait for human ==
    loop until a human is close enough and for more than 0.5 sec.
    openni -> image_dispatcher : /openni/rgb/image_color/compressed (CompressedImage)
    openni -> image_dispatcher : /openni/depth/points (PointCloud2)
    image_dispatcher -> body_tracking : /devine/image/body_tracking (CompressedImage)
    body_tracking -> dialog_control : /devine/body_tracking (String)
    end


    == Extract image data  ==
    image_dispatcher -> feature_extraction : /devine/image/features_extraction (CompressedImage)
    scene_finder -> image_segmentation : /devine/image/segmentation (CompressedImage)
    image_segmentation -> GuessWhat : /devine/objects (SegmentedImage)
    feature_segmentation -> GuessWhat : /devine/image_features (Float64MultiArray)



    @enduml


Specifics for each node can be found below.

Image Dispatcher
=============================


Include links to ROS module section

Include brief description

Body tracking
==============

Include links to ROS module section

Include brief description


Segmentation
============

Include links to ROS module section

Include brief description

Feature extraction
===================

Include links to ROS module section

Include brief description

Depth mask
===========

Where does this fit into our architecture?


