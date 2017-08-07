#!/bin/bash
#data_path=~/Datasets/ARC/20_7/
data_path=/bc5/objects/test_epsom/
calibration_path=$(eval rospack find arc_calibrate_turntable)/results/table.tf
echo calibration path is $calibration_path


#iterate over the subfolders
for folder in $data_path/*
do

    folder=${folder%*/}   
    folder_name=${folder##*/}
    #check that we are not at  any of {background     cam_T_obj.tf    capture.txt}
    if [ "$folder_name" != "background" ] && [ "$folder_name" != "cam_T_obj.tf" ] && [ "$folder_name" != "capture.txt" ]
    then

        if [ "$folder_name" == "epsom_salts" ]
        then

            echo ${folder_name}
            # for each object list all the views
            for view in ${folder}/*
            do

            #if [[ -d "${view}" ]] ; then

                    #copy the calibration file to <current folder>/cam_T_obj.tf

                    echo at folder $folder
                    cp $calibration_path $view/cam_T_obj.tf

                    view_name=${view##*/}
                    if [ "$view_name" != "grasps.yaml" ]
                    then
                        echo $view
                        rosrun arc_registration register_views $view
                        rosrun  arc_registration align_tfs_views $view
                        #--prefix "gdb --args"
                    fi

                done
             #fi

        fi


    fi


done
