#!/bin/python
import time
import boto3
import csv
import numpy as np
from pms3003 import PMSensor
import RPi.GPIO as GPIO
import dht11

# run aws configure and set AWS Access Key ID and AWS Secret Access Key
# set up project path for csv generation
path = '/project/path/'

# get dht11 data
# initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

# read dht11 data using pin 4
instance = dht11.DHT11(pin = 4)		#verify if pin is correct

# measure hum and temp (n-1)-times
n = 10
data_dht = []

# loop over
for i in range(1,n/2):
        while True:
                result = instance.read()
                if result.is_valid():
                        data_dht = np.append([data_dht], [result.temperature, result.humidity])
                        break
        time.sleep(2)

# call a PMSensor class
# 0 for indoor sensing, 1 for outdoor
pm = PMSensor(1)

# get PM1, PM2.5, PM10 values
data = pm.read_pm()

# reject outliers
data = data[np.all(np.abs((data - np.mean(data, axis=0))) <= 2 * np.std(data, axis=0), axis=1)]
data = data[np.array(data[:,2], dtype=float)/np.array(data[:,1], dtype=float) < 2]
data = data[np.array(data[:,2], dtype=float) > np.array(data[:,1], dtype=float)]

# get the pm average as an int
pm1, pm25, pm10 = np.mean(data, axis=0, dtype=int)

# continue measuring hum and temp
for i in range(1,n/2):
        while True:
                result = instance.read()
                if result.is_valid():
                        data_dht = np.append([data_dht], [result.temperature, result.humidity])
                        break
        time.sleep(2)

# GPIO cleanup
GPIO.cleanup()

# reshape and reject outliers
data_dht = data_dht.reshape((n-2,2))
data_dht = data_dht[np.all(np.abs((data_dht - np.mean(data_dht, axis=0))) <= 2 * np.std(data_dht, axis=0), axis=1)]

# get the dht average as an int
temp, hum = np.mean(data_dht, axis=0, dtype=int)

# get current timestamp
pm_date = (time.strftime('%Y-%m-%d ') + time.strftime('%H:%M:%S'))

# write to csv
with open(path + 'pm-archive.csv','a+') as f:
 writer = csv.writer(f)
 writer.writerow([pm_date,pm1,pm25,pm10,temp,hum])

# write to dynamodb table
dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('pms3003')	#verify table name

try:
        # insert the data
        table.put_item(
                Item={
                'device' : 'pms3003',
                'dt' : pm_date,
                'pm1' : pm1,
                'pm25' : pm25,
                'pm10' : pm10,
                'temp' : temp,
                'hum' : hum,
            }
        )

except Exception:
        # write to csv in case of error
        with open(path + 'pm-not-loaded.csv','a+') as fn:
         writer = csv.writer(fn)
         writer.writerow([pm_date,pm1,pm25,pm10,temp,hum])
		
	# SNS notification about an unsuccessful load
        #sns = boto3.resource('sns')
        #topic = sns.Topic('arn::pastehere::')
        #response = topic.publish(
        #       Message='Problem loading data on: ' + pm_date,
        #       Subject='PMS3003 Unsuccessful Data Load',
        #       MessageStructure='string'
        #)
