#!/bin/bash

containers=`docker container list | awk '{ print $11 }'`

for c in $containers;
do
	docker container kill $c
	docker container rm $c
done

./clean-volumes.sh
