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
			<div class="widget-cpu-frequency"><label id="cpu1-frequency">-</label></div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu1-progress" class="cpu-progress" value="0" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 2 [<label id="cpu2-percentage">-</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu2-frequency">-</label></div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu2-progress" class="cpu-progress" value="0" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 3 [<label id="cpu3-percentage">-</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu3-frequency">-</label></div>
		</div>
		<div class="widget-cpu-row">
			<progress id="cpu3-progress" class="cpu-progress" value="0" max="100"></progress>
		</div>
	</div>
	<div class="widget-cpu">
		<div class="widget-cpu-row">
			<div class="widget-cpu-percentage">CPU 4 [<label id="cpu4-percentage">-</label>]</div>
			<div class="widget-cpu-frequency"><label id="cpu4-frequency">-</label></div>
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
		// Stats of CPU 1
		this.$cpu1_percentage = this._shadowRoot.getElementById("cpu1-percentage");
		this.$cpu1_progress = this._shadowRoot.getElementById("cpu1-progress");
		this.$cpu1_frequency = this._shadowRoot.getElementById("cpu1-frequency");
		// Stats of CPU 2
		this.$cpu2_percentage = this._shadowRoot.getElementById("cpu2-percentage");
		this.$cpu2_progress = this._shadowRoot.getElementById("cpu2-progress");
		this.$cpu2_frequency = this._shadowRoot.getElementById("cpu2-frequency");
		// Stats of CPU 3
		this.$cpu3_percentage = this._shadowRoot.getElementById("cpu3-percentage");
		this.$cpu3_progress = this._shadowRoot.getElementById("cpu3-progress");
		this.$cpu3_frequency = this._shadowRoot.getElementById("cpu3-frequency");
		// Stats of CPU 4
		this.$cpu4_percentage = this._shadowRoot.getElementById("cpu4-percentage");
		this.$cpu4_progress = this._shadowRoot.getElementById("cpu4-progress");
		this.$cpu4_frequency = this._shadowRoot.getElementById("cpu4-frequency");

		this.stats_data = null;
	}

	set data(value)
	{
		this.stats_data = value;
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

		if (this.stats_data)
		{
			// Stats of CPU 1
			this.$cpu1_percentage.innerHTML = this.stats_data.status[1].values[1].value;
			this.$cpu1_progress.value = parseInt(this.stats_data.status[1].values[1].value);
			this.$cpu1_frequency.innerHTML = this.stats_data.status[1].values[2].value;
			// Stats of CPU 2
			this.$cpu2_percentage.innerHTML = this.stats_data.status[2].values[1].value;
			this.$cpu2_progress.value = parseInt(this.stats_data.status[2].values[1].value);
			this.$cpu2_frequency.innerHTML = this.stats_data.status[2].values[2].value;
			// Stats of CPU 3
			if(this.stats_data.status[3].message === "OFF")
			{
				this.$cpu3_percentage.innerHTML = "Off";
				this.$cpu3_progress.value = 0;
				this.$cpu3_frequency.innerHTML = "-";
			}
			else
			{
				this.$cpu3_percentage.innerHTML = this.stats_data.status[3].values[1].value;
				this.$cpu3_progress.value = parseInt(this.stats_data.status[3].values[1].value);
				this.$cpu3_frequency.innerHTML = this.stats_data.status[3].values[2].value;
			}
			// Stats of CPU 4
			if(this.stats_data.status[4].message === "OFF")
			{
				this.$cpu4_percentage.innerHTML = "Off";
				this.$cpu4_progress.value = 0;
				this.$cpu4_frequency.innerHTML = "-";
			}
			else
			{
				this.$cpu4_percentage.innerHTML = this.stats_data.status[4].values[1].value;
				this.$cpu4_progress.value = parseInt(this.stats_data.status[4].values[1].value);
				this.$cpu4_frequency.innerHTML = this.stats_data.status[4].values[2].value;
			}
		}
	}
	
}

window.customElements.define('jetson-stats', JetsonStats);