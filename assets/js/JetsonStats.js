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
/*
var JetsonStats = (function(rosWebObject)
{
	// Subscribing to a Topic of Sonar Range Center
	let listenerDiagnostics = new ROSLIB.Topic(
	{
		ros : rosWebObject.Ros,
		name : '/diagnostics',
		messageType : 'diagnostic_msgs/DiagnosticArray'
	});
	// Add a callback to be called every time a message is published on this topic.
	listenerDiagnostics.subscribe(function(message)
	{
		//document.getElementById("USValueCenter").innerText = message.range;
		console.log(message);
	});
});
*/

const template = document.createElement('template');

template.innerHTML = `
<style>
.jetson-stats
{
	border: 1px solid black;
}
.jetson-stats h3
{
	text-align: center;
}
.widget-cpu
{
	padding: 0 5px 5px 5px;
}
.widget-cpu-row
{
	width: 100%;
}
.widget-cpu-percentage
{
	width: 50%;
	float: left;
}
.widget-cpu-frequency
{
	text-align: right;
}
.cpu-progress
{
	width: 100%;
}
</style>
<div class="jetson-stats">
	<h3>-</h3>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 1 [<label id="cpu1-percentage">-</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu1-frequency">150</label> MHz</div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu1-progress" class="cpu-progress" value="32" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 2 [<label id="cpu2-percentage">25%</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu2-frequency">150</label> MHz</div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu2-progress" class="cpu-progress" value="25" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 3 [<label id="cpu3-percentage">Off</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu3-frequency">0</label> MHz</div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu3-progress" class="cpu-progress" value="0" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 4 [<label id="cpu4-percentage">Off</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu4-frequency">0</label> MHz</div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu4-progress" class="cpu-progress" value="0" max="100"></progress>
		</div>
	</div>
</div>
`;

class JetsonStats extends HTMLElement
{
	constructor()
	{
		super();
		this._shadowRoot = this.attachShadow({ mode: 'open' });
		this._shadowRoot.appendChild(template.content.cloneNode(true));

		this.$title = this._shadowRoot.querySelector('h3');
		this.$cpu1_percentage = this._shadowRoot.getElementById("cpu1-percentage");
	}

	set data(value)
	{
		this.setAttribute('data', value);
	}

	static get observedAttributes()
	{
		return ['title', 'data'];
	}

	attributeChangedCallback(name, oldVal, newVal)
	{
		this.render();
	}

	render()
	{
		this.$title.innerHTML = this.title;

		console.log(this.data);

		this.$cpu1_percentage.innerHTML = this.data.status[1].message;
	}
	
}

window.customElements.define('jetson-stats', JetsonStats);