/*
 * The MIT License (MIT)
 *
 * This file is part of the jetbot_ros_webconsole package (https://github.com/bobboteck/jetbot_ros_webconsole).
 Copyright (c) 2020 Roberto D'Amico (Bobboteck).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

var RosWeb = (function()
{
	const STATE = { NotConnected: 0, Connected: 1, Closed: 2, ConnectionError: 3, Error: 4 };

	let self = this;
	let connectionError = false;

	function Status()
	{
		this.Status = "Not connected";
		this.Message = "";
		this.Code = STATE.NotConnected;
	}

	this.ConnectionStatus = new Status();

	// Initialize ROS Lib JS
	this.Ros = new ROSLIB.Ros();

	// If there is an error on the backend, an 'error' emit will be emitted.
	this.Ros.on("error", function(error) 
	{
		connectionError = true;
		
		self.ConnectionStatus.Status = "Error";
		self.ConnectionStatus.Message = error;
		self.ConnectionStatus.Code = STATE.Error;
	});

	// Find out exactly when we made a connection.
	this.Ros.on("connection", function() 
	{
		self.ConnectionStatus.Status = "Connected";
		self.ConnectionStatus.Message = "";
		self.ConnectionStatus.Code = STATE.Connected;
	});

	this.Ros.on("close", function() 
	{
		if(!connectionError)
		{
			self.ConnectionStatus.Status = "Closed";
			self.ConnectionStatus.Message = "";
			self.ConnectionStatus.Code = STATE.Closed;
		}
	});

	this.Connection = function(wsAddress)
	{
		try
		{
			// Create a connection to the rosbridge WebSocket server.
			this.Ros.connect(wsAddress);
		}
		catch(error)
		{
			this.ConnectionStatus.Status = "Error";
			this.ConnectionStatus.Message = error;
			this.ConnectionStatus.Code = STATE.ConnectionError;
		}
	};
});