extern crate serde;
extern crate serde_json;

use std::fs;
use std::io;
use std::collections;
use std::cell;

#[derive(Serialize, Deserialize)]
pub struct Object {
    pub id: String,
    pub rad: f64,
    pub pos: (f64, f64, f64),
}

#[derive(Serialize, Deserialize)]
pub struct ObjectPosition {
    pub id: String,
    pub pos: (f64, f64, f64),
}

#[derive(Serialize, Deserialize)]
pub struct SimulationProc {
    objects: Vec<Object>,
    frames: cell::RefCell<collections::HashMap<usize, Vec<ObjectPosition>>>,
}

impl SimulationProc {
    pub fn new(objects: Vec<Object>) -> Self {
        SimulationProc {
            objects: objects,
            frames: cell::RefCell::<collections::HashMap<usize, Vec<ObjectPosition>>>::new(
                collections::HashMap::<usize, Vec<ObjectPosition>>::new(),
            ),
        }
    }

    pub fn add_frame(&self, frame_order: usize, frame: Vec<ObjectPosition>) {
        let mut frames = self.frames.borrow_mut();
        for k in frames.keys() {
            if frame_order <= *k {
                panic!("frame order incompetible");
            }
        }
        frames.insert(frame_order, frame);
    }

    pub fn save(&self, file_path: &str) {
        let f = fs::File::create(file_path).unwrap();
        let buf_writer = io::BufWriter::new(f);
        serde_json::to_writer(buf_writer, self).unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::prelude::*;

    #[test]
    fn test_add_frame() {
        let sim_proc = SimulationProc::new(Vec::<Object>::new());
        let first = 0_usize;
        let second = 1_usize;

        sim_proc.add_frame(
            first,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (0., 0., 0.),
                },
                ObjectPosition {
                    id: "s1".to_string(),
                    pos: (1., 0., 0.),
                },
                ObjectPosition {
                    id: "s2".to_string(),
                    pos: (2., 0., 0.),
                },
            ],
        );
        sim_proc.add_frame(
            second,
            vec![
                ObjectPosition {
                    id: "s1".to_string(),
                    pos: (0., 3., 0.),
                },
                ObjectPosition {
                    id: "s2".to_string(),
                    pos: (0., 4., 0.),
                },
            ],
        );

        assert_eq!(sim_proc.frames.borrow()[&first][0].id, "s0");
        assert_eq!(sim_proc.frames.borrow()[&first][0].pos, (0., 0., 0.));
        assert_eq!(sim_proc.frames.borrow()[&first][1].id, "s1");
        assert_eq!(sim_proc.frames.borrow()[&first][1].pos, (1., 0., 0.));
        assert_eq!(sim_proc.frames.borrow()[&first][2].id, "s2");
        assert_eq!(sim_proc.frames.borrow()[&first][2].pos, (2., 0., 0.));

        assert_eq!(sim_proc.frames.borrow()[&second][0].id, "s1");
        assert_eq!(sim_proc.frames.borrow()[&second][0].pos, (0., 3., 0.));
        assert_eq!(sim_proc.frames.borrow()[&second][1].id, "s2");
        assert_eq!(sim_proc.frames.borrow()[&second][1].pos, (0., 4., 0.));
    }

    #[test]
    #[should_panic]
    fn panic_on_frame_order_duplicated() {
        let sim_proc = SimulationProc::new(Vec::<Object>::new());
        let first = 0_usize;

        sim_proc.add_frame(
            first,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (0., 0., 0.),
                },
            ],
        );
        sim_proc.add_frame(
            first,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (0., 3., 0.),
                },
            ],
        );
    }

    #[test]
    #[should_panic]
    fn panic_on_frame_order_disturbed() {
        let sim_proc = SimulationProc::new(Vec::<Object>::new());
        let first = 0_usize;
        let second = 1_usize;

        sim_proc.add_frame(
            second,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (0., 0., 0.),
                },
            ],
        );
        sim_proc.add_frame(
            first,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (0., 3., 0.),
                },
            ],
        );
    }

    #[test]
    fn test_save() {
        let sim_proc = SimulationProc::new(
            vec![
                ("s0".to_string(), 2., (0., 0., 0.)),
                ("s1".to_string(), 3., (1., 0., 0.)),
            ].into_iter()
                .map(move |(id, r, pos)| Object {
                    id: id,
                    rad: r,
                    pos: pos,
                })
                .collect::<Vec<Object>>(),
        );

        let first = 0_usize;
        let second = 1_usize;
        sim_proc.add_frame(
            first,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (1., 0., 0.),
                },
                ObjectPosition {
                    id: "s1".to_string(),
                    pos: (2., 0., 0.),
                },
            ],
        );
        sim_proc.add_frame(
            second,
            vec![
                ObjectPosition {
                    id: "s0".to_string(),
                    pos: (2., 1., 0.),
                },
                ObjectPosition {
                    id: "s1".to_string(),
                    pos: (3., 2., 0.),
                },
            ],
        );

        clean_file(|x| {
            sim_proc.save(&x.file_path);

            let file = fs::File::open(&x.file_path).unwrap();
            let mut buf_reader = io::BufReader::new(file);
            let mut buf = String::new();
            buf_reader.read_to_string(&mut buf).unwrap();

            println!("\nsaved json file\n{}\n", buf);
            let patterns = vec![
            "{\
                \"objects\":[{\"id\":\"s0\",\"rad\":2.0,\"pos\":[0.0,0.0,0.0]},{\"id\":\"s1\",\"rad\":3.0,\"pos\":[1.0,0.0,0.0]}],\
                \"frames\":{\
                    \"0\":[{\"id\":\"s0\",\"pos\":[1.0,0.0,0.0]},{\"id\":\"s1\",\"pos\":[2.0,0.0,0.0]}],\
                    \"1\":[{\"id\":\"s0\",\"pos\":[2.0,1.0,0.0]},{\"id\":\"s1\",\"pos\":[3.0,2.0,0.0]}]\
                }\
            }",
            "{\
                \"objects\":[{\"id\":\"s0\",\"rad\":2.0,\"pos\":[0.0,0.0,0.0]},{\"id\":\"s1\",\"rad\":3.0,\"pos\":[1.0,0.0,0.0]}],\
                \"frames\":{\
                    \"1\":[{\"id\":\"s0\",\"pos\":[2.0,1.0,0.0]},{\"id\":\"s1\",\"pos\":[3.0,2.0,0.0]}],\
                    \"0\":[{\"id\":\"s0\",\"pos\":[1.0,0.0,0.0]},{\"id\":\"s1\",\"pos\":[2.0,0.0,0.0]}]\
                }\
            }",
            ];
            assert!(buf == patterns[0] || buf == patterns[1]);
        });
    }

    struct TestFixture {
        file_path: String,
    }

    fn clean_file<F: Fn(&TestFixture)>(f: F) {
        let tf = TestFixture {
            file_path: "test.json".to_string(),
        };
        f(&tf);
        fs::remove_file(&tf.file_path).unwrap();
    }
}
