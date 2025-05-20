#!/bin/bash

volumes=`docker volume list | awk '{ print $2 }'`

for v in $volumes;
do
	docker volume rm $v
done
