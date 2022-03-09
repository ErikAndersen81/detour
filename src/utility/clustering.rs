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
    /// Agglomerative clustering
    /// The cluster distance is determined by the minimum distance of two elements from separate clusters.
    /// Initially, each element is considered a cluster containing only the element.
    /// Clusters are merged bottom up until the distance of all clusters exceeds the `threshold`.
    pub fn new(distance_matrix: Vec<Vec<f64>>, threshold: f64) -> Self {
        let mut clusters: Vec<HashSet<MatrixIdx>> = vec![];
        for (idx, _) in distance_matrix.iter().enumerate() {
            let mut set = HashSet::new();
            set.insert(idx as MatrixIdx);
            clusters.push(set);
        }
        let mut c = Clustering {
            distance_matrix,
            clusters,
            threshold,
        };
        c.prep_distance_matrix();
        c.partition();
        c
    }

    fn cluster_distance(&self, a: ClusterIdx, b: ClusterIdx) -> f64 {
        let elements_a: Vec<MatrixIdx> = self.clusters[a].iter().copied().collect();
        let elements_b: Vec<MatrixIdx> = self.clusters[b].iter().copied().collect();
        let mut dist = f64::INFINITY;
        for a in elements_a {
            for b in elements_b.iter() {
                dist = dist.min(self.distance_matrix[a][*b]);
            }
        }
        dist
    }

    /// We don't want to merge clusters with themselves, so we set internal distance to infinite.
    fn prep_distance_matrix(&mut self) {
        let n: MatrixIdx = self.distance_matrix.len();
        for i in 0..n {
            self.distance_matrix[i][i] = f64::INFINITY;
        }
    }

    fn partition(&mut self) {
        let mut merging = true;
        while merging {
            merging = false;
            let n: MatrixIdx = self.clusters.len();
            'outer: for a in 0..n {
                for b in (a + 1)..n {
                    let dist = self.cluster_distance(a, b);
                    if dist < self.threshold {
                        self.merge_clusters(a, b);

                        merging = true;
                        break 'outer;
                    }
                }
            }
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
