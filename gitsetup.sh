#! /bin/bash

email='airoprojectsx@gmail.com'
username='airoprojects'
repository="robot-programming-nc"
token="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"

git remote set-url origin https://${token}@github.com/${username}/${repository}
git remote -v
