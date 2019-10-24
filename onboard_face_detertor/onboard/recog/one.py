import sys
import os
import requests

target_file = sys.argv[1]

with open(target_file, 'rb') as f:
  r = requests.post('http://localhost:8888/upload', files={'file': f})
  os.remove(target_file)

  if "True" in r.text:
    f = target_file.split('.jpg', 1)
    d = f[0].split('_', 5)

    f = open("../drift.txt","w")

    f.write("{}_{}_{}_{}".format(d[1], d[2], d[3], d[4]))

    f.close()