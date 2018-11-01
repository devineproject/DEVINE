#!/bin/bash
use_gpu='false'
use_gui='false'
use_audio='false'

while test $# -gt 0; do
    case "$1" in
        -h|--help)
            echo "Run the DEVINE Docker image"
            echo " "
            echo "Note: might need to 'xhost +' first"
            echo " "
            echo "options:"
            echo "-h, --help                show brief help"
            echo "-g, --gpu                 use the GPU"
            echo "-d, --display             hook the GUI server to the host"
            echo "-a, --audio               hook audio to the host"
            exit 0;;
        -g|--gpu)
            use_gpu='true'
            shift;;
        -d|--display)
            use_gui='true'
            shift;;
        -a|--audio)
            use_audio='true'
            shift;;
    esac
done


if $use_gpu
then
render_args='--runtime=nvidia'
else
render_args='-e LIBGL_ALWAYS_SOFTWARE=1'
fi

if $use_gui
then
display_args='-e DISPLAY=:0 -v /tmp/.X11-unix:/tmp/.X11-unix'
else
display_args=''
fi

if $use_audio
then
audio_args='--device /dev/snd -v /etc/asound.conf:/etc/asound.conf:ro'
else
audio_args=''
fi

mount_points='-v /home/felixlabelle/git_repo3/DEVINE/tests:/usr/src/DEVINE/tests -v /home/felixlabelle/git_repo3/DEVINE/src/image_processing/src/devine_image_processing:/usr/src/DEVINE/src/image_processing/src/devine_image_processing -v /home/felixlabelle/git_repo3/DEVINE/src/devine/launch:/usr/src/DEVINE/src/devine/launch -v /home/felixlabelle/bag_data:/usr/src/DEVINE/data'
CMD="sudo docker run -p 9090:9090 -p 8080:8080 -it --rm -e QT_X11_NO_MITSHM=1 \
    $mount_points $render_args $display_args $audio_args \
    devine bash"

echo $CMD
eval $CMD
