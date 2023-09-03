#! /bin/bash

email='airoprojectsx@gmail.com'
username='airoprojects'
repository="robot-programming-nc"
token="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"

git remote set-url origin https://${token}@github.com/${username}/${repository}
git remote -v
