use std::time::{Instant, Duration};
use std::fs::File;
use std::io::prelude::*;
use std::collections::HashMap;

pub struct TimeProfiler<'a> {
    start_time: Instant,
    last_checkpoint_time: Instant,
    total_elapsed_times_memo: HashMap<&'a str, Vec<Duration>>,
    elapsed_times_memo: HashMap<&'a str, Vec<Duration>>,
    inserted_order: Vec<&'a str>,
}

fn fmt_duration(d: &Duration) -> String {
    if d.as_secs() == 0 {
        format!("{}us", d.subsec_nanos() as f64 / 1000_f64)
    } else {
        format!("{}s,{}us", d.as_secs(), d.subsec_nanos() as f64 / 1000_f64)
    }
}

impl<'a> TimeProfiler<'a> {
    pub fn new() -> Self {
        let now = Instant::now();
        TimeProfiler{
            start_time: now.clone(),
            last_checkpoint_time: now,
            total_elapsed_times_memo: HashMap::new(),
            elapsed_times_memo: HashMap::new(),
            inserted_order: Vec::<&'a str>::new(),
        }
    }

    pub fn reset_time(&mut self) {
        let now = Instant::now();
        self.start_time = now.clone();
        self.last_checkpoint_time = now;
    }

    pub fn reset_memo(&mut self) {
        self.total_elapsed_times_memo.clear();
        self.elapsed_times_memo.clear();
    }

    pub fn check(&mut self, msg: &'a str) {
        let now = Instant::now();

        let total_elapsed_time = now.duration_since(self.start_time);
        if self.total_elapsed_times_memo.contains_key(msg) {
            self.total_elapsed_times_memo.get_mut(msg).unwrap().push(total_elapsed_time);
        } else {
            self.total_elapsed_times_memo.insert(msg, vec![total_elapsed_time]);
        }

        let elapsed_time_from_last_checkpoint = now.duration_since(self.last_checkpoint_time);
        if self.elapsed_times_memo.contains_key(msg) {
            self.elapsed_times_memo.get_mut(msg).unwrap().push(elapsed_time_from_last_checkpoint);
        } else {
            self.elapsed_times_memo.insert(msg, vec![elapsed_time_from_last_checkpoint]);
        }

        if !self.inserted_order.contains(&msg) {
            self.inserted_order.push(msg)
        }

        self.last_checkpoint_time = Instant::now();
        self.start_time = self.start_time + self.last_checkpoint_time.duration_since(now); // cancel processing time loss
    }

    pub fn output(&self) -> String {
        let mut out = String::new();
        for msg in self.inserted_order.iter() {
            let total_elapsed_mean = mean_duration(self.total_elapsed_times_memo.get(msg).unwrap());
            let elapsed_mean = mean_duration(self.elapsed_times_memo.get(msg).unwrap());

            out.push_str(&format!(
                "------------------------------------------------------------------------------------------------------------------------\n{}  (elapsed time --- total: {}, since last checkpoint: {})\n------------------------------------------------------------------------------------------------------------------------\n",
                msg, fmt_duration(&total_elapsed_mean), fmt_duration(&elapsed_mean),
            ))
        }
        out
    }

    pub fn save(&self, file_path: &str) {
        let out_str = self.output();
        let mut file = File::create(file_path).unwrap();
        file.write_all(out_str.as_bytes()).unwrap();
    }
}

fn mean_duration(v: &Vec<Duration>) -> Duration {
    v.iter().fold(Duration::new(0, 0), |acc, x| acc + *x / v.len() as u32)
}
