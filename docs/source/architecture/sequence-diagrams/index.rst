UML Sequence Diagrams
#####################

Start Game
==========
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
        participant human_finder
        participant image_dispatcher
        participant dialog_control
        participant scene_finder
        participant snips
    end box
    box "Robot Embedded" #LightBlue
        participant irl1
    end box

     'UML'
    title Start Game

     == Wait for human ==
    loop until a human is close enough and for more than 0.5 sec.
    openni -> image_dispatcher : /openni/rgb/image_color/compressed (CompressedImage)
    openni -> image_dispatcher : /openni/depth/points (PointCloud2)
    dialog_control -> human_finder : /devine/human_finder (LookAtHumanGoal)
     loop until human is found
    image_dispatcher -> body_tracking : /devine/image/body_tracking (CompressedImage)
    body_tracking -> human_finder : /devine/body_tracking (String)
    human_finder -> dialog_control : /devine/human_finder (LookAtHumanActionFeedback)
    human_finder -> irl1 : /jn0/neck_controller/follow_joint_trajectory (JointTrajectoryPoint)
    end

    == Ask human if wants to play ==
     loop until discussion is complete
    dialog_control -> snips : /devine/tts/query (TtsQuery)
    snips -> dialog_control : /devine/tts/answer (TtsAnswer)
    end
    dialog_control -> image_dispatcher : /devine/player_name (String)

    == Find scene to play ==
    loop until 2 AprilTags are found
    image_dispatcher -> scene_finder : /devine/zone_detection (CompressedImage)
    scene_finder -> irl1 : /jn0/neck_controller/follow_joint_trajectory (JointTrajectoryPoint)
    end
    scene_finder -> image_dispatcher: /devine/scene_found (Bool)

    @enduml

Play Game
=========
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
    box "External GPU" #LightGreen
            participant image_processing
    end box
    box "Robot CPU" #FFFFFF
        participant guesswhat
        participant image_dispatcher
        participant snips
    end box

    'UML'
    title Play Game

    == Prepare data for GuessWhat?! ==
    image_dispatcher -> image_processing : /devine/image/segmentation (CompressedImage)
    image_processing -> guesswhat : /devine/image_features (VGG16Features)
    image_processing -> guesswhat : /devine/objects (SegmentedImage)

    == Play GuessWhat?! ==
    loop until discussion is completed
    guesswhat -> snips : /devine/tts/query (TtsQuery)
    snips -> guesswhat : /devine/tts/answer (TtsAnswer)
    end

    @enduml

End of Game
===========
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
        participant guesswhat
        participant image_dispatcher
        participant snips
        participant pos_lib
        participant openni
        participant robot_control
        participant dialog_control
    end box
    box "Robot Embedded" #LightBlue
        participant irl1
    end box

    'UML'
    title End of Game

    == Point guessed object ==
    guesswhat -> dialog_control : /devine/objects_confidence (Float64MultiArray)
    guesswhat -> dialog_control : /devine/guess_category (String)
    guesswhat -> pos_lib : /devine/guess_location/image (PointStamped)
    openni -> pos_lib: /openni/depth/points (PointCloud2)
    pos_lib -> robot_control : /devine/guess_location/world (PoseStamped)
    note left: referenced from 'base_link'
    robot_control -> irl1 : /jn0/<left/right>_arm_controller/follow_joint_trajectory (JointTrajectoryPoint)
    robot_control -> dialog_control : /devine/robot/is_pointing (Bool)

    == Ask player if guess is good ==
    dialog_control -> snips : /devine/tts/query (TtsQuery)
    snips -> dialog_control : /devine/tts/answer (TtsAnswer)

    == Emote based on game result ==
    dialog_control -> robot_control : /devine/guesswhat_succeeded (Bool)
    guesswhat -> robot_control : /devine/objects_confidence (Float64MultiArray)
    guesswhat -> robot_control : /devine/object_guess_success (Bool)
    robot_control -> irl1 : /jn0/emo_intensity (EmoIntensity)

    @enduml