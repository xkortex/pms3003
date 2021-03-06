#!/bin/python
import boto3
import pandas as pd

# write to dynamodb table
dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('pms3003')
table_tmp = dynamodb.Table('pms3003tmp')

# function for scan
def dynamo_scan(table):
 response = table.scan(table)
 data = response['Items']
 df = pd.DataFrame(data, columns=['dt','pm1','pm25','pm10','hum','temp']).sort_values('dt').set_index('dt')
 df.index = pd.to_datetime(df.index)
 return df

# scan the table
df = dynamo_scan(table_tmp)

# loop over and put items with sort index
for i in range(df.shape[0]):
 if(df.hum.isnull().values[i] == True):
  Item={
   'device' : 'pms3003',
   'dt' : str(df.index[i]),
   'pm1' : int(df.pm1[i]),
   'pm25' : int(df.pm25[i]),
   'pm10' : int(df.pm10[i]),
  } 
 else:
  Item={
   'device' : 'pms3003',
   'dt' : str(df.index[i]),
   'pm1' : int(df.pm1[i]),
   'pm25' : int(df.pm25[i]),
   'pm10' : int(df.pm10[i]),
   'temp' : int(df.temp[i]),
   'hum' : int(df.hum[i]),
  }
 table.put_item(
         Item = Item
 )
