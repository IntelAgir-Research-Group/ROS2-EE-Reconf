#!/bin/bash

containers=`docker container ls -a | awk '{ print $1 }'`

for c in $containers;
do
	docker container kill $c
	docker container rm $c
done

./clean-volumes.sh
