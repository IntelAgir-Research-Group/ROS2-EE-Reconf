#!/bin/bash

containers=`docker container ls -a | awk '{ print $1 }'`

for c in $containers;
do
	echo "Limpando container $c"
	docker container kill $c
	docker container rm $c
done

./clean-volumes.sh
