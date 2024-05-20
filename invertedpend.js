//author: Ryan Huang

// fetch the canvas element from html file
var canvas = document.getElementById("theCanvas");
var ctx = canvas.getContext('2d');

// resize the canvas to the size of the current window
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// canvas dimensions
const canvasW = canvas.width;
 		canvasH = canvas.height;

// physical constants
const g = 9.81;						// acceleration of gravity
		m = 1;							// pendulum mass
		M = 5;							// cart mass
		L = 200; 						// rod length
		dt = 0.05;						// differential time element
		ySlider = canvasH * 0.6;				// the ground
		cartW = 30;						// cart width
    		cartH = 20;						// cart height
	 	barThickness = 5;				// thickness of the slider bar

// controller variables
let control = true;
let kp = 350;
let ki = 5;
let kd = 10;
let integral = 0;
let prevError = 0;


class InvertedPendulum {
	constructor(theta, theta_dot, cartX, cartY, cartVx) {
		// all the state variables and initial conditions are set here
		this.cartX = canvasW/2;
		this.cartY = ySlider;
		this.cartVx = 0;
		this.theta = Math.PI*(0.48);
		this.theta_dot = 0;
	}

	showPend() {
		// draw the pendulum, slider, the rigid rod, and the sliding bar
		let pend = canvas.getContext('2d');
		pend.beginPath();
		pend.fillStyle = 'white';
		pend.arc(this.cartX - L*Math.cos(this.theta), this.cartY - L*Math.sin(this.theta), 6, 0, Math.PI * 2, true);
		pend.fill();
		pend.closePath();

		let cart = canvas.getContext('2d');
		cart.beginPath();
		cart.fillStyle = 'white';
		cart.fillRect(this.cartX- cartW/2, ySlider - cartH/2, cartW, cartH);
		cart.fill();
		cart.closePath();

		let line = canvas.getContext('2d');
		line.beginPath();
		line.moveTo(this.cartX, this.cartY);
		line.lineTo(this.cartX - L*Math.cos(this.theta), this.cartY - L*Math.sin(this.theta));
		line.strokeStyle = 'white';
		line.stroke();

		let ground = canvas.getContext('2d');
		ground.beginPath();
		ground.fillStyle = "#f1f0f0";
		ground.fillRect(0, ySlider, canvasW, barThickness);
		ground.stroke();

	}

	calcPhysics(x, x_dot, theta, theta_dot, F, control) {
		// calculate the new linear and angular accelerations

		// force adjusted by the correction force from PID
		let adjustedForce = this.PID(control, prevError);

		F += adjustedForce;

		// equations derived from the euler lagrange equations
		// equations of motion

		let x_ddot = (F - m * L * Math.pow(theta_dot,2) * Math.cos(theta) + m * g * Math.cos(theta) * Math.sin(theta)) / (M + m - m * Math.pow(Math.sin(theta),2));
		let theta_ddot = -((g * Math.cos(theta))/L) - ((Math.sin(theta) * x_ddot)/L);

		// my attempt at 4th order runge kutta
		let k1 = x_ddot;
		let k2 = (F - m * L * Math.pow(theta_dot + (0.5) * dt * k1,2) * Math.cos(theta + (0.5) * dt * k1) + m * g * Math.cos(theta + (0.5) * dt * k1) * Math.sin(theta + (0.5) * dt * k1)) / (M + m - m * Math.pow(Math.sin(theta + (0.5) * dt * k1),2));
		let k3 = (F - m * L * Math.pow(theta_dot + (0.5) * dt * k2,2) * Math.cos(theta + (0.5) * dt * k2) + m * g * Math.cos(theta + (0.5) * dt * k2) * Math.sin(theta + (0.5) * dt * k2)) / (M + m - m * Math.pow(Math.sin(theta + (0.5) * dt * k2),2));
		let k4 = (F - m * L * Math.pow(theta_dot + dt * k3,2) * Math.cos(theta + dt * k3) + m * g * Math.cos(theta + dt * k3) * Math.sin(theta + dt * k3)) / (M + m - m * Math.pow(Math.sin(theta + dt * k3),2));
		x_ddot = x_ddot + (1/6) * dt * (k1 +2*k2 + 2*k3 + k4);

		let kt1 = theta_ddot;
		let kt2 = -((g * Math.cos(theta + (0.5) * dt * kt1))/L) - ((Math.sin(theta + (0.5) * dt * kt1) * x_ddot)/L);
		let kt3 = -((g * Math.cos(theta + (0.5) * dt * kt2))/L) - ((Math.sin(theta + (0.5) * dt * kt2) * x_ddot)/L);
		let kt4 = -((g * Math.cos(theta + dt * kt3))/L) - ((Math.sin(theta + dt * kt3) * x_ddot)/L);
		theta_ddot = theta_ddot + (1/6) * dt * (kt1 +2*kt2 + 2*kt3 + kt4);

		return [x_ddot, theta_ddot];
	}

	movePend(force, control) {
		let state = this.calcPhysics(this.cartX, this.cartVx, this.theta, this.theta_dot, force, control);
		// console.log(state);

		// update the velocity and angular velocity
		this.cartVx += state[0] * dt - Math.max(0,0);
		this.theta_dot += state[1] * dt - Math.max(0,0);

		// update the position and angle
		this.cartX += this.cartVx * dt + 0.5 * state[0] * Math.pow(dt,2);
		this.theta += this.theta_dot * dt + 0.5 * state[1] * Math.pow(dt,2);

		if (this.theta > Math.PI * 2) {
			this.theta -= Math.PI * 2;
		}
		else if (this.theta < -(Math.PI * 2)) {
			this.theta += Math.PI * 2;
		}

		// bounce off the walls with half the velocity
		if (this.cartX - cartW/2 < 0) {
			this.cartVx *= -0.5;
			this.theta_dot *= 0.5;
			this.cartX = cartW/2;
			// this.cartX = canvasW - cartW/2;
		}
		else if (this.cartX + cartW/2 > canvasW) {
			this.cartVx *= -0.5;
			this.theta_dot *= 0.5;
			this.cartX = canvasW - cartW/2;
			// this.cartX = cartW/2;
		}
	}

	PID(control, prevError) {
		if (control == true) {
			let error = Math.PI/2 - this.theta;		// error term

			if (error > Math.abs(Math.PI/4)) { // give up if the angle is too severe
				return 0;
			}

			let deriv = (error - prevError) / dt;	// derivative term
			integral += error * dt						// integral term

			let correction = -kp * error - ki * integral - kd * deriv;	// PID algorithm

			prevError = error;

			// uncomment the return 0 to disable to controller permanantly
			// comment out the correction to disable the controller

			return correction;
			// return 0;
		}
		else {
			return 0;
		}
	}

}

myPend = new InvertedPendulum();


// apply a force to the cart
var appliedForce = 0;

// right arrow key applies a force to the right
// left arrow key applies a force to the left
document.onkeydown = checkKey;
document.onkeyup = noKey;

// check for any key downs to modify the value of appliedForce
function checkKey(e) {
	 e = e || window.event;

	 if (e.keyCode == '37') {			// left arrow key
		 console.log("left key down");
		 appliedForce = -35;
	 }
	 else if (e.keyCode == '39') {	// right arrow key
		 console.log("right key down");
		 appliedForce = 35;
	 }
	 if (e.keyCode == '32') {	// space bar
		 console.log("controller off");
		 control = false;
	 }
}

// check for no key presses to reset applied force back to 0
function noKey(e) {
	e = e || window.event;

	if (e.keyCode == '37' || e.keyCode == '39' || e.keyCode == '32') {
		console.log("controller on")
		appliedForce = 0;
		control = true;
	}
}

document.addEventListener('keydown', checkKey);
document.addEventListener('keyup', noKey);


setInterval(function() {
		const clear = canvas.getContext("2d");
		clear.clearRect(0, 0, canvasW, canvasH);

		console.log(control);
		myPend.showPend();
		myPend.movePend(appliedForce, control);

}, 0);
