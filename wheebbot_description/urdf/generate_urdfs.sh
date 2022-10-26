#!/bin/sh

xacro wheebbot.urdf.xacro -o generated/wheebbot.urdf 
xacro wheebbot_full.urdf.xacro floating_joint:=true -o generated/wheebbot_full.urdf
 
