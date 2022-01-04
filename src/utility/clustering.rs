use std::collections::HashSet;
#[allow(dead_code)]
type ClusterIdx = usize;
type MatrixIdx = usize;

pub struct Clustering {
    distance_matrix: Vec<Vec<f64>>,
    pub clusters: Vec<HashSet<MatrixIdx>>,
    threshold: f64,
}

impl Clustering {
    pub fn new(distance_matrix: Vec<Vec<f64>>, threshold: f64) -> Self {
        let clusters: Vec<HashSet<MatrixIdx>> = vec![];
        let mut c = Clustering {
            distance_matrix,
            clusters,
            threshold,
        };
        c.partition();
        c
    }

    fn partition(&mut self) {
        let n: MatrixIdx = self.distance_matrix.len();
        for a in 0..n {
            for b in (a + 1)..n {
                let dist = self.distance_matrix[a][b];
                let a: ClusterIdx = self.get_cluster_idx(a);
                let b: ClusterIdx = self.get_cluster_idx(b);
                if dist < self.threshold {
                    self.merge_clusters(a, b);
                }
            }
        }
    }

    fn get_cluster_idx(&mut self, value: MatrixIdx) -> ClusterIdx {
        // If the value is not in any cluster a new one is created
        for (idx, cluster) in self.clusters.iter().enumerate() {
            if cluster.contains(&value) {
                return idx;
            }
        }
        self.add_to_cluster(&[value], None);
        self.clusters.len() - 1
    }

    fn add_to_cluster(&mut self, values: &[usize], cluster: Option<ClusterIdx>) {
        let mut set: HashSet<usize> = HashSet::new();
        for value in values {
            set.insert(*value);
        }
        if let Some(cluster) = cluster {
            self.clusters[cluster] = self.clusters[cluster]
                .union(&set)
                .copied()
                .collect::<HashSet<usize>>();
        } else {
            self.clusters.push(set);
        }
    }

    fn merge_clusters(&mut self, a: ClusterIdx, b: ClusterIdx) {
        if a == b {
            return;
        }
        let (a, b) = (a.min(b), a.max(b));
        let b: HashSet<usize> = self.clusters.remove(b);
        let a: HashSet<usize> = self.clusters.remove(a);
        let c: HashSet<usize> = a.union(&b).copied().collect();
        self.clusters.push(c);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn add_to_new_cluster_test() {
        let mut c = Clustering {
            distance_matrix: vec![],
            threshold: 1.,
            clusters: vec![],
        };
        c.add_to_cluster(&[1usize, 2usize], None);
        assert_eq!(c.clusters.len(), 1);
        assert!(c.clusters[0].contains(&1usize));
        assert!(c.clusters[0].contains(&2usize));
    }

    #[test]
    fn add_to_existing_cluster_test() {
        let cls = HashSet::from([0usize, 1usize]);
        let mut c = Clustering {
            distance_matrix: vec![],
            threshold: 1.,
            clusters: vec![cls],
        };
        c.add_to_cluster(&[1usize, 2usize], Some(0usize));
        assert_eq!(c.clusters.len(), 1);
        assert!(c.clusters[0].contains(&0usize));
        assert!(c.clusters[0].contains(&1usize));
        assert!(c.clusters[0].contains(&2usize));
    }

    #[test]
    fn merge_clusters_test() {
        let cls1 = HashSet::from([0usize, 1usize]);
        let cls2 = HashSet::from([2usize, 3usize]);
        let mut c = Clustering {
            distance_matrix: vec![],
            threshold: 1.,
            clusters: vec![cls1, cls2],
        };
        c.merge_clusters(0, 1);
        assert_eq!(c.clusters.len(), 1);
        assert!(c.clusters[0].contains(&0usize));
        assert!(c.clusters[0].contains(&1usize));
        assert!(c.clusters[0].contains(&2usize));
        assert!(c.clusters[0].contains(&3usize));
    }

    #[test]
    fn simple() {
        let distances = vec![vec![0., 1., 2.], vec![1., 0., 1.], vec![2., 1., 0.]];
        let c = Clustering::new(distances, 0.5);
        assert_eq!(c.clusters.len(), 3);
    }

    #[test]
    fn simple1() {
        let distances = vec![vec![0., 1., 2.], vec![1., 0., 1.], vec![2., 1., 0.]];
        let c = Clustering::new(distances, 1.1);
        assert_eq!(c.clusters.len(), 1);
    }
}
