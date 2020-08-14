//!
//! Boids
//!
//! TODO: Boids disappearing, likely from rules being implemented
//! functionally. Make imperative?
//! TODO: Implement rules on flock, not Boid

extern crate piston_window;

use rand::Rng;
use std::time::{SystemTime,UNIX_EPOCH};
use std::path::Path;
use piston_window::*;

// Piston settings
const WINDOW_SIZE: u32 = 800;
const GFX_CONTEXT_OFFSET: f64 = 0.0 as f64;
const MILLIS_PER_FRAME: u128 = 10;
const BLACK: [f32;4] = [0.0, 0.0, 0.0, 1.0];
const WHITE: [f32;4] = [1.0; 4];
const SQUARE_SIZE: f64 = 2.0;

// Simulation settings
const NBOIDS: usize = 300;   // Number of boids
const NPREDS: usize = 0;     // Number of predator boids
const MAXSPEED: f64 = 2.5;   // Max speed of boids
const MAXDIST: f64  = 25.0;  // Max distance to apply rules
const MINSEP: f64   = 10.0;

const ALIGNMENT_WEIGHT: f64  = 0.125;
const SEPARATION_WEIGHT: f64 = 1.0;
const COHESION_WEIGHT: f64   = 0.01;
const OBSTACLE_WEIGHT: f64   = 1.0;


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
        let mag = self.get_magnitude();
        self.x = self.x / mag;
        self.y = self.y / mag;
    }
    fn set_magnitude(&mut self, magnitude: f64) {
        self.normalize();
        self.x = self.x * magnitude;
        self.y = self.y * magnitude;
    }
    fn get_magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
    fn get_distance(&self, vector: &Vector) -> f64 {
        let mut vector = Vector { x: vector.x , y: vector.y };
        vector.subvec(self);
        vector.get_magnitude()
    }
    fn dot(&self, vector: Vector) -> f64 {
        self.x * vector.x + self.y * vector.y
    }
}

#[derive(Debug)]
struct Boid {
    id: usize,
    predator: bool,
    maxspeed: f64,
    location: Vector,
    velocity: Vector,
}

impl Boid {
    /// Maintain similar velocity as nearby boids
    fn compute_alignment(&self, boids: &[Boid]) -> Vector {
        // Imperative implementation
        let mut nearby = 0;
        let mut alignment = [0.0 ; 2];
        for i in 0 .. boids.len() {
            if self.id != boids[i].id && 
               self.location.get_distance(&boids[i].location) < MAXDIST {
                alignment[0] += boids[i].velocity.x;
                alignment[1] += boids[i].velocity.y;
            }
        }
        let mut alignment = Vector { x: alignment[0] , y: alignment[1] };
        alignment.multscalar(1.0);
        alignment

        // Functional implementation
        // let alignment: (f64,f64) = boids.iter()
        //                                 .filter(|b| b.id != self.id && 
        //                                    (b.location.x - self.location.x).abs() < MAXDIST &&
        //                                    (b.location.y - self.location.y).abs() < MAXDIST)
        //                                 .map(|b| (b.velocity.x , b.velocity.y))
        //                                 .fold((0.0, 0.0), |sum, v| {
        //                                     let sum = (sum.0 + v.0, sum.1 + v.1);
        //                                     sum
        //                                 });
        // let mut alignment = Vector { x: alignment.0 , y: alignment.1 };
        // alignment.multscalar(1.0);
        // alignment
    }
                

    /// Maintain some separation from other boids
    fn compute_separation(&self, boids: &[Boid]) -> Vector {
        // Imperative implementation
        let nearby = 0;
        let mut separation = [0.0 ; 2];
        for i in 0 .. boids.len() {
            if self.location.get_distance(&boids[i].location) < MINSEP &&
               self.id != boids[i].id {
                separation[0] += self.location.x - boids[i].location.x;
                separation[1] += self.location.y - boids[i].location.y;
                // separation.subvec(&boids[i].location);
            }
        }
        let separation: Vector = Vector { x: separation[0] , y: separation[1] };
        separation

        // Functional implementation
        // let separation: (f64,f64) = boids.iter()
        //                                  .filter(|b| b.id != self.id)
        //                                  .map(|b| (b.location.x, b.location.y))
        //                                  .filter(|l| (l.0 - self.location.x).abs() < MINSEP &&
        //                                              (l.1 - self.location.y).abs() < MINSEP)
        //                                  .map(|l| { (self.location.x - l.0 ,
        //                                              self.location.y - l.1) })
        //                                  .fold((0.0, 0.0), |sum, l| {
        //                                      let sum = (sum.0 + l.0, sum.1 + l.1);
        //                                      sum
        //                                  });
        // let mut separation: Vector = Vector { x: separation.0, y: separation.1 };
        // separation.multscalar(1.0);
        // separation
    }

    /// Move towards center of mass of nearby boids
    fn compute_cohesion(&self, boids: &[Boid]) -> Vector {
        let mut nearby = 0;
        let mut cohesion: Vector = Vector { x: 0.0, y: 0.0 };
        for i in 0 .. boids.len() {
            if (boids[i].location.x - self.location.x).abs() < MAXDIST &&
               (boids[i].location.y - self.location.y).abs() < MAXDIST &&
               (boids[i].id != self.id) {
                cohesion.addvec(&boids[i].location);
                nearby += 1;
            }
        }
        if nearby > 0 { cohesion.multscalar(1.0 / nearby as f64); }
        cohesion.subvec(&self.location);
        cohesion.multscalar(1.0);
        cohesion
    }

    /// Boids are repelled from bounds and obstacles
    fn compute_obstacle(&self) -> Vector {
        // TODO: Obstacle vector increases proportionally to boundary closeness
        let mut dir: (f64,f64) = (0.0,0.0);
        if self.location.x < MINSEP {
            // dir.0 = 100.0 / (self.location.x);
            // dir.0 = 10.0;
            dir.0 = (MINSEP - self.location.x).exp();
        }
        if self.location.y < MINSEP {
            dir.1 = (MINSEP - self.location.y).exp();
        }
        if self.location.x > (WINDOW_SIZE as f64 - MINSEP) {
            dir.0 = -10.0;
        }
        if self.location.y > (WINDOW_SIZE as f64 - MINSEP) {
            dir.1 = -10.0;
        }

        Vector { x: dir.0, y: dir.1 }
    }

    fn update_velocity(&mut self, v: &Vector) {
        self.velocity.addvec(v);
    }
    fn update_position(&mut self) {
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
    /// Generates flock containing boids and predators
    fn generate_flock(&mut self, n_boids: usize, n_preds: usize) {
        let mut rng = rand::thread_rng();
        self.flock = Vec::with_capacity(n_boids + n_preds);
        for i in 0 .. (n_boids + n_preds) {
            self.add_boid(Boid {
                id: i,
                predator: false,
                maxspeed: MAXSPEED,
                location: Vector {x: rng.gen_range(0.0, WINDOW_SIZE as f64),
                                  y: rng.gen_range(0.0, WINDOW_SIZE as f64)},
                velocity: Vector {x: 1.0, y: 0.0},
            });
        }
        for i in 0 .. n_preds {
            self.flock[i].predator = true;
            self.flock[i].maxspeed = MAXSPEED + 1.0;
        }
    }

    /// Adds a boid to the flock
    fn add_boid(&mut self, boid: Boid) {
        self.flock.push(boid);
    }

    /// Moves simulation forward a single step by applying rules to each
    /// boid and updating position
    fn take_step(&mut self) {
        for i in 0 .. self.flock.len() {
            let mut algn = self.flock[i].compute_alignment(&self.flock);
            let mut sepr = self.flock[i].compute_separation(&self.flock);
            let mut cohn = self.flock[i].compute_cohesion(&self.flock);
            let obst = self.flock[i].compute_obstacle();
            algn.multscalar(ALIGNMENT_WEIGHT);
            sepr.multscalar(SEPARATION_WEIGHT);
            cohn.multscalar(COHESION_WEIGHT);
            let mut sum = Vector{x: 0.0, y: 0.0};
            sum.addvec(&sepr);
            sum.addvec(&algn);
            sum.addvec(&cohn);
            sum.addvec(&obst);
            self.flock[i].update_velocity(&sum);
            self.flock[i].limit_speed();
            self.flock[i].update_position();
        }
    }
}


fn main() {
    let mut f = Flock { flock: vec![] };
    f.generate_flock(NBOIDS, 0);

    let opengl = OpenGL::V3_2;
    let mut window: PistonWindow = WindowSettings::new("boids", [WINDOW_SIZE; 2])
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

