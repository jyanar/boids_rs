//!
//! Boids
//!

extern crate piston_window;

use rand::Rng;
use std::time::{SystemTime,UNIX_EPOCH};
use std::path::Path;
use piston_window::*;



const NBOIDS: usize = 200;    // Number of boids
const NPREDS: usize = 0;     // Number of predator boids
const MAXSPEED: f64 = 1.5;    // Max speed of boids
const MAXFORCE: f64 = 1.0;    // Max force to apply
const MAXDIST: f64  = 5.0;  // Max distance to apply rules
const MINSEP: f64   = 3.0;

const ALIGN_WEIGHT: f64 = 1.0;
const SEPAR_WEIGHT: f64 = 1.0;
const COHSN_WEIGHT: f64 = 1.0;

const WINDOW_HEIGHT: usize = 2024;
const WINDOW_WIDTH: usize  = 1024;

// Piston Window stuff
const WINDOW_SIZE: u32 = 1024;
//const GFX_CONTEXT_OFFSET: f64 = (WINDOW_SIZE / 2) as f64;
const GFX_CONTEXT_OFFSET: f64 = 0.0 as f64;
const MILLIS_PER_FRAME: u128 = 1;
const BLACK: [f32;4] = [0.0, 0.0, 0.0, 1.0];
const WHITE: [f32;4] = [1.0; 4];
const SQUARE_SIZE: f64 = 2.0;

#[derive(Debug)]
struct Vector {
    x: f64,
    y: f64,
}

impl Vector {
    fn addvec(&mut self, vector: &Vector) {
        self.x = self.x + vector.x;
        self.y = self.y + vector.y;
    }
    fn subvec(&mut self, vector: &Vector) {
        self.x = self.x - vector.x;
        self.y = self.y - vector.y;
    }
    fn addscalar(&mut self, scalar: f64) {
        self.x = self.x + scalar;
        self.y = self.y + scalar;
    }
    fn subscalar(&mut self, scalar: f64) {
        self.x = self.x + scalar;
        self.y = self.y + scalar;
    }
    fn multscalar(&mut self, scalar: f64) {
        self.x = self.x * scalar;
        self.y = self.y * scalar;
    }
    fn normalize(&mut self) {
        self.x = self.x / self.get_magnitude();
        self.y = self.y / self.get_magnitude();
    }
    fn set_magnitude(&mut self, magnitude: f64) {
        self.normalize();
        self.x = self.x * magnitude;
        self.y = self.y * magnitude;
    }
    fn dot(&self, vector: Vector) -> f64 {
        self.x * vector.x + self.y * vector.y
    }
    fn get_magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
}

#[derive(Debug)]
struct Boid {
    id: usize,
    predator: bool,
    maxspeed: f64,
    maxforce: f64,
    location: Vector,
    velocity: Vector,
    acceleration: Vector,
}

impl Boid {
    /// Boids maintain similar velocities
    fn compute_alignment(&self, boids: &[Boid]) -> Vector {
        let alignment: (f64,f64) = boids.iter()
                                        .filter(|b| b.id != self.id && 
                                           (b.location.x - self.location.x).abs() < MAXDIST &&
                                           (b.location.y - self.location.y).abs() < MAXDIST)
                                        .map(|b| (b.velocity.x , b.velocity.y))
                                        .fold((0.0, 0.0), |sum, v| {
                                            let sum = (sum.0 + v.0, sum.1 + v.1);
                                            sum
                                        });
        let alignment = Vector {x: alignment.0 / 8.0, y: alignment.1 / 8.0};
        alignment
    }

    /// Boids maintain some separation
    fn compute_separation(&self, boids: &[Boid]) -> Vector {
        let separation: (f64,f64) = boids.iter()
                                         .filter(|b| b.id != self.id)
                                         .map(|b| (b.location.x, b.location.y))
                                         .filter(|l| (l.0 - self.location.x).abs() < MINSEP &&
                                                     (l.1 - self.location.y).abs() < MINSEP)
                                         .map(|l| { (self.location.x - l.0 ,
                                                     self.location.y - l.1) })
                                         .fold((0.0, 0.0), |sum, l| {
                                             let sum = (sum.0 + l.0, sum.1 + l.1);
                                             sum
                                         });
        let separation: Vector = Vector { x: separation.0 / 10.0, y: separation.1 / 10.0};
        separation
    }

    /// Boids move towards the center of mass
    fn compute_cohesion(&self, boids: &[Boid]) -> Vector {
        let n_boids = boids.len() as f64;
        // To compute the boids center of mass, we filter out boids
        // which are further away than MAXDIST, and compute their average
        // (x,y) position
        let locs_sum: (f64,f64) = boids.iter()
                                       .filter(|b| b.id != self.id)
                                       .map(|b| (b.location.x, b.location.y))
                                       .filter(|l| (l.0 - self.location.x).abs() < MAXDIST &&
                                                   (l.1 - self.location.y).abs() < MAXDIST)
                                       .fold((0.0, 0.0), |sum, l| {
                                           let sum = (sum.0 + l.0 , sum.1 + l.1);
                                           sum
                                       });
        let centroid: (f64,f64) = (locs_sum.0/(n_boids - 1.0), locs_sum.1/(n_boids - 1.0));
        // Vector designating direction from self.location to the centroid
        let cohesion: Vector = Vector { x: (centroid.0 - self.location.x) / 100.0 ,
                                        y: (centroid.1 - self.location.y) / 100.0 };
        cohesion
    }

    /// Boids are repelled from bounds and obstacles
    fn compute_obstacle(&self) -> Vector {
        let mut dir: (f64,f64) = (0.0,0.0);
        if self.location.x < MINSEP {
            // dir.0 = 100.0 / self.location.x;
            dir.0 = 10.0;
        }
        if self.location.y < MINSEP {
            // dir.1 = 100.0 / self.location.y;
            dir.1 = 10.0;
        }
        if self.location.x > (WINDOW_WIDTH as f64 - MINSEP) {
            // dir.0 = -1.0 * (100.0 / (WINDOW_WIDTH as f64 - MINSEP));
            dir.0 = -10.0;
        }
        if self.location.y > (WINDOW_HEIGHT as f64 - MINSEP) {
            // dir.1 = -1.0 * (100.0 / (WINDOW_HEIGHT as f64 - MINSEP));
            dir.1 = -10.0;
        }

        Vector { x: dir.0, y: dir.1 }
    }

    fn apply_force(&mut self, force: &Vector) {
        self.acceleration.addvec(force);
    }
    fn update_velocity(&mut self, v: &Vector) {
        self.velocity.addvec(v);
    }
    fn take_step(&mut self) {
        self.location.addvec(&self.velocity);
    }
    fn limit_speed(&mut self) {
        if self.velocity.get_magnitude() > MAXSPEED {
            self.velocity.set_magnitude(MAXSPEED);
        }
    }
}


struct Flock {
   flock: Vec<Boid>,
}

impl Flock {
    fn generate_flock(&mut self, n_boids: usize, n_preds: usize) {
        let mut rng = rand::thread_rng();
        self.flock = Vec::with_capacity(n_boids + n_preds);
        for i in 0 .. (n_boids + n_preds) {
            self.add_boid(Boid {
                id: i,
                predator: false,
                maxspeed: MAXSPEED, maxforce: MAXFORCE,
                location: Vector {x: rng.gen_range(0.0, 1000.0),
                                  y: rng.gen_range(0.0, 20.0)},
                velocity: Vector {x: 1.0, y: 0.0},
                acceleration: Vector {x: 0.0, y: 0.0},
            });
        }
        for i in 0 .. n_preds {
            self.flock[i].predator = true;
            self.flock[i].maxspeed = MAXSPEED + 1.0;
            self.flock[i].maxforce = MAXFORCE + 1.0;
        }
    }

    fn add_boid(&mut self, boid: Boid) {
        self.flock.push(boid);
    }

    fn take_step(&mut self) {
        // Apply all three rules to the flock
        for i in 0 .. self.flock.len() {
            let mut algn = self.flock[i].compute_alignment(&self.flock);
            let mut sepr = self.flock[i].compute_separation(&self.flock);
            let mut cohn = self.flock[i].compute_cohesion(&self.flock);
            algn.multscalar(ALIGN_WEIGHT);
            sepr.multscalar(SEPAR_WEIGHT);
            cohn.multscalar(COHSN_WEIGHT);
            let obst = self.flock[i].compute_obstacle();
            let mut sum = Vector{x: 0.0, y: 0.0};
            sum.addvec(&sepr);
            sum.addvec(&algn);
            sum.addvec(&cohn);
            sum.addvec(&obst);
            self.flock[i].update_velocity(&sum);
            self.flock[i].limit_speed();
            self.flock[i].take_step();
        }
    }
}


fn main() {
    // Generate flock with 10 boids
    let mut f = Flock { flock: vec![] };
    f.generate_flock(NBOIDS, 0);


    // let v: Vector = f.flock[0].compute_cohesion(&f.flock);
    // let v: Vector = f.flock[0].compute_separation(&f.flock);
    // let v: Vector = f.flock[0].compute_alignment(&f.flock);
    // println!("{:?}", v);

    let opengl = OpenGL::V3_2;
    let mut window: PistonWindow = WindowSettings::new("Boids", [WINDOW_SIZE; 2])
        .exit_on_esc(true)
        .graphics_api(opengl)
        .build()
        .unwrap();

    let mut previous_update = UNIX_EPOCH;

    while let Some(e) = window.next() {
        if previous_update.elapsed()
                          .map(|d| d.as_millis())
                          .unwrap_or(0) > MILLIS_PER_FRAME {
            f.take_step();
            previous_update = SystemTime::now();
        }

        window.draw_2d(&e, |context, graphics, _| {
            clear(BLACK, graphics);
            let context = context.trans(GFX_CONTEXT_OFFSET, GFX_CONTEXT_OFFSET);
            for i in 0 .. f.flock.len() {
                rectangle(WHITE, [f.flock[i].location.x,
                                  f.flock[i].location.y,
                                  SQUARE_SIZE, SQUARE_SIZE],
                                  context.transform,
                                  graphics);
            }
        });
    }
}

