#! /bin/bash

email='airoprojectsx@gmail.com'
username='airoprojects'
repository="robot-programming-nc"
token="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"

git remote set-url origin https://${token}@github.com/${username}/${repository}
git remote -v
