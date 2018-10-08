sudo docker images | grep '\(<none>\s\+\)\(<none>\s\+\)\([0-9a-f]\+\)' | sed 's/\(<none>\s\+\)\(<none>\s\+\)\([0-9a-f]\+\).*/\3/g' | xargs sudo docker rmi
