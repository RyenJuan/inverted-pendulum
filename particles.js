//author: Ryan Huang

// fetch the canvas element from html file
var canvas = document.getElementById("theCanvas");
var ctx = canvas.getContext('2d');

// resize the canvas to the size of the current window
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

// simulate particles going past the boundaries
let padding = 0;

// canvas dimensions
let canvasW = canvas.width + padding;
let canvasH = canvas.height + padding;

// number of particles to draw
let n = canvasW / 13;

//minimum distance to draw an edge between particles
let d = 150;

function getRandomHexColor(minHex, maxHex) {
    const hexToRGB = hex => ({
        r: parseInt(hex.substring(1, 3), 16),
        g: parseInt(hex.substring(3, 5), 16),
        b: parseInt(hex.substring(5, 7), 16),
    });

    const RGBToHex = rgb => `#${(1 << 24 | rgb.r << 16 | rgb.g << 8 | rgb.b).toString(16).slice(1)}`;

    const minRGB = hexToRGB(minHex);
    const maxRGB = hexToRGB(maxHex);

    const randomRGB = () => ({
        r: Math.floor(Math.random() * (maxRGB.r - minRGB.r + 1) + minRGB.r),
        g: Math.floor(Math.random() * (maxRGB.g - minRGB.g + 1) + minRGB.g),
        b: Math.floor(Math.random() * (maxRGB.b - minRGB.b + 1) + minRGB.b),
    });


    return RGBToHex(randomRGB());
}


class Particle {
	// create a particle object with a random location and random velocity
	constructor(x, y, vx, vy, color) {
		this.x = Math.random() * canvasW;
		this.y = Math.random() * canvasH;
		this.vx = Math.random() * 4 - 2;
		this.vy = Math.random() * 4 - 2;
		this.color = Math.floor(Math.random()*16777215).toString(16);
	}

	showParticle() {
		// show the particle on the canvas
		ctx.beginPath();
		ctx.fillStyle = "#" + this.color;
		ctx.arc(this.x, this.y, 3, 0, Math.PI * 2, true);
		ctx.globalAlpha = 1;
		ctx.closePath();
		ctx.fill();
	}

	moveParticle() {
		// account for wall collisions
		// if (this.x < -padding || this.x > canvasW) {
		// 	this.vx *= -1;
		// }
		// if (this.y < -padding || this.y > canvasH) {
		// 	this.vy *= -1;
		// }

		// particle teleports to the other side once it hits a boundary

		if (this.x < -padding) {
			this.x = canvasW;
		}
		if (this.x > canvasW) {
			this.x = -padding;
		}
		if (this.y < -padding) {
			this.y = canvasH;
		}
		if (this.y > canvasH) {
			this.y = -padding;
		}

		this.x += this.vx;
		this.y += this.vy;
	}

	connectEdges(index, particles) {
		// measure the distance between a given particle and all other particles
		// connect any particles within a certain distance
		for(let i = 0; i < n; i++) {
			let dist = Math.hypot(particles[index].x - particles[i].x, particles[index].y - particles[i].y);
			if (dist < d) {
				let line = canvas.getContext('2d');
				line.beginPath();
				line.moveTo(particles[index].x, particles[index].y);
				line.lineTo(particles[i].x, particles[i].y);
				line.strokeStyle = "#" + this.color;
				line.globalAlpha = 1 - (dist / d);
				line.stroke();
			}
		}
	}
}

// array to store each particle object
const particles = [];

// create all initial particle objects and push them to the array
for(let i = 0; i < n; i++) {
	particles.push(new Particle());
}


// update the positions of all particles every 40 ms
setInterval(function() {
	//remove the previous particles
	const clear = canvas.getContext("2d");
	clear.clearRect(0, 0, canvasW, canvasH);

	//show, move, and connect all the particles
	for(let i = 0; i < particles.length; i++) {
		particles[i].showParticle();
		particles[i].moveParticle();
		particles[i].connectEdges(i, particles);
	}
}, 40);
