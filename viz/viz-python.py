# -*- coding: utf-8 -*-
#!/bin/python
import time
import pandas as pd
import boto3
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Output, Event
import plotly
import plotly.graph_objs as go

# config boto3 first
# get the service resource
dynamodb = boto3.resource('dynamodb')

# instantiate a table resource object 
table = dynamodb.Table('pms3003')

# initiate a dash app
app = dash.Dash()

# set formatting
colors = {
    'background': '#ffffff',
    'text': '#111101'
}

# actual dash app layout definition
app.layout = html.Div(style={'backgroundColor': colors['background']}, children=[

	# title
    html.H1(
        children='Zanieczyszczenie powietrza',
        style={
            'textAlign': 'center',
            'color': colors['text']
        }
    ),
	
	# graph
    dcc.Graph(
        id='live-graph',
		animate = True
    ),
	
	# graph update handler
	dcc.Interval(
            id='graph-update',
            interval=1*1000
        ),
		
	# last updated note
	html.Div(children='Ostatni pomiar wykonano ' + str(1), style={
        'textAlign': 'right',
        'color': colors['text'],
		'fontSize': 12
    })
])

# app callback for graph update
@app.callback(Output('live-graph', 'figure'),
              events=[Event('graph-update', 'interval')])
			  
# function for the graph udpate
def update_graph():

	# scan the table
	response = table.scan()
	data = response['Items']
	
	# create a pandas dataframe, wrangle the data
	df = pd.DataFrame(data, columns=['dt','pm1','pm25','pm10']).sort_values('dt').set_index('dt').astype(int)
	df.index = pd.to_datetime(df.index)

	# trace0 - pm1
	trace0 = plotly.graph_objs.Scatter(
		x=df.index,
		y=df['pm1'],
		name='pm1',
		mode= 'lines+markers'
	)
	
	# trace1 - pm2.5
	trace1 = plotly.graph_objs.Scatter(
		x=df.index,
		y=df['pm25'],
		name='pm2.5',
		mode= 'lines+markers'
	)
	
	# trace2 - pm10
	trace2 = plotly.graph_objs.Scatter(
		x=df.index,
		y=df['pm10'],
		name='pm10',
		mode= 'lines+markers'
	)
	
	# combine lines
	data = [trace0, trace1, trace2]

	# return quasi-live data for graph's figure attribute
	return {'data': data,'layout' : go.Layout(yaxis = dict(title = "µg/m3"))}

# run
if __name__ == '__main__':
    app.run_server(debug=True)
