using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using SlimDX;

namespace PlanetGeneration {
    static class Program {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main() {

            var planet = new Planet(new GenerationSettings {
                Subdivisions = 20,
                DistortionLevel = 1,
                PlateCount = 36,
                OceanicRate = 0.7f,
                HeatLevel = 1.0f,
                MoistureLevel = 1.0f,
                Seed = 0
            });


            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());


        }
    }

    public struct GenerationSettings {
        public int Subdivisions { get; set; }
        public int DistortionLevel { get; set; }
        public int PlateCount { get; set; }
        public float OceanicRate { get; set; }
        public float HeatLevel { get; set; }
        public float MoistureLevel { get; set; }

        public int Seed { get; set; }
    }

    public class Planet {
        public int Seed { get; set; }
        public GenerationSettings Settings { get; set; }

        public Planet(GenerationSettings settings) {
            Settings = settings;

            Seed = settings.Seed;

            var random = new Random(Seed);

            var mesh = GeneratePlanetMesh(settings.Subdivisions, settings.DistortionLevel, random);
        }

        private object GeneratePlanetMesh(int subdivisions, int distortionLevel, Random random) {
            var mesh = GenerateSubdividedIcosahedron(subdivisions);
            Console.WriteLine("Generating subdivided icosahedron");

            var totalDistortion = mesh.Edges.Count * distortionLevel;
            var remainingIterations = 6;
            while (remainingIterations > 0) {
                var iterationDistortion = totalDistortion / remainingIterations;
                totalDistortion -= iterationDistortion;
                DistortMesh(mesh, iterationDistortion, random);
                RelaxMesh(mesh, 0.5);
                --remainingIterations;
            }

            return false;
        }

        private bool DistortMesh(Mesh mesh, int degree, Random random) {
            var i = 0;
            while (true) {
                if (i >= degree) {
                    break;
                }
                var consecutiveFailedAttempts = 0;
                var edgeIndex = random.Next(0, mesh.Edges.Count);
                while (!ConditionalRotateEdge(mesh, edgeIndex, RotationPredicate)) {
                    if (++consecutiveFailedAttempts >= mesh.Edges.Count) {
                        return false;
                    }
                    edgeIndex = (edgeIndex + 1) % mesh.Edges.Count;
                }
                ++i;
            }
            return true;
        }

        private static bool ConditionalRotateEdge(Mesh mesh, int edgeIndex, Func<Node, Node, Node, Node, bool> predicate) {
            var edge = mesh.Edges[edgeIndex];
            var face0 = mesh.Faces[edge.Faces[0]];
            var face1 = mesh.Faces[edge.Faces[1]];
            var farNodeFaceIndex0 = GetFaceOppositeNodeIndex(face0, edge);
            var farNodeFaceIndex1 = GetFaceOppositeNodeIndex(face1, edge);
            var newNodeIndex0 = face0.Nodes[farNodeFaceIndex0];
            var oldNodeIndex0 = face0.Nodes[(farNodeFaceIndex0 + 1)%3];
            var newNodeIndex1 = face1.Nodes[farNodeFaceIndex1];
            var oldNodeIndex1 = face1.Nodes[(farNodeFaceIndex1 + 1)%3];
            var oldNode0 = mesh.Nodes[oldNodeIndex0];
            var oldNode1 = mesh.Nodes[oldNodeIndex1];
            var newNode0 = mesh.Nodes[newNodeIndex0];
            var newNode1 = mesh.Nodes[newNodeIndex1];
            var newEdgeIndex0 = face1.Edges[(farNodeFaceIndex1 + 2)%3];
            var newEdgeIndex1 = face0.Edges[(farNodeFaceIndex0 + 2)%3];
            var newEdge0 = mesh.Edges[newEdgeIndex0];
            var newEdge1 = mesh.Edges[newEdgeIndex1];

            if (!predicate(oldNode0, oldNode1, newNode0, newNode1)) {
                return false;
            }

            oldNode0.Edges.RemoveAt(oldNode0.Edges.IndexOf(edgeIndex));
            oldNode1.Edges.RemoveAt(oldNode1.Edges.IndexOf(edgeIndex));
            newNode0.Edges.Add(edgeIndex);
            newNode1.Edges.Add(edgeIndex);

            edge.Nodes[0] = newNodeIndex0;
            edge.Nodes[1] = newNodeIndex1;

            newEdge0.Faces.RemoveAt(newEdge0.Faces.IndexOf(edge.Faces[1]));
            newEdge1.Faces.RemoveAt(newEdge1.Faces.IndexOf(edge.Faces[0]));
            newEdge0.Faces.Add(edge.Faces[0]);
            newEdge1.Faces.Add(edge.Faces[1]);

            oldNode0.Faces.RemoveAt(oldNode0.Faces.IndexOf(edge.Faces[1]));
            oldNode1.Faces.RemoveAt(oldNode1.Faces.IndexOf(edge.Faces[0]));
            newNode0.Faces.Add(edge.Faces[1]);
            newNode1.Faces.Add(edge.Faces[0]);

            face0.Nodes[(farNodeFaceIndex0 + 2)%3] = newNodeIndex1;
            face1.Nodes[(farNodeFaceIndex1 + 2)%3] = newNodeIndex0;

            face0.Edges[(farNodeFaceIndex0 + 1)%3] = newEdgeIndex0;
            face1.Edges[(farNodeFaceIndex1 + 1)%3] = newEdgeIndex1;
            face0.Edges[(farNodeFaceIndex0 + 2)%3] = edgeIndex;
            face1.Edges[(farNodeFaceIndex1 + 2)%3] = edgeIndex;

            return true;
        }

        private static int GetFaceOppositeNodeIndex(Face face, Edge edge) {
            if (face.Nodes[0] != edge.Nodes[0] && face.Nodes[0] != edge.Nodes[1]) return 0;
            if (face.Nodes[1] != edge.Nodes[0] && face.Nodes[1] != edge.Nodes[1]) return 1;
            if (face.Nodes[2] != edge.Nodes[0] && face.Nodes[2] != edge.Nodes[1]) return 2;
            throw new Exception("Cannot find node of given face that is not also a node of given edge");
        }

        private static bool RotationPredicate(Node old0, Node old1, Node new0, Node new1) {
            if (new0.Faces.Count >= 7 || new1.Faces.Count >= 7 || old0.Faces.Count <= 5 || old1.Faces.Count <= 5) {
                return false;
            }

            var oldEdgeLength = Vector3.Distance(old0.Position, old1.Position);
            var newEdgeLength = Vector3.Distance(new0.Position, new1.Position);
            var ratio = oldEdgeLength / newEdgeLength;
            if (ratio >= 2 || ratio <= 0.5) {
                return false;
            }
            var v0 = (old1.Position - old0.Position) / oldEdgeLength;
            var v1 = Vector3.Normalize(new0.Position - old0.Position);
            var v2 = Vector3.Normalize(new1.Position - old0.Position);
            if (Vector3.Dot(v0, v1) < 0.2f || Vector3.Dot(v0, v2) < 0.2f) {
                return false;
            }
            v0 = -v0;
            var v3 = Vector3.Normalize(new0.Position - old1.Position);
            var v4 = Vector3.Normalize(new1.Position - old1.Position);
            if (Vector3.Dot(v0, v3) < 0.2f || Vector3.Dot(v0, v4) < 0.2f) {
                return false;
            }
            return true;
        }

        private static Mesh GenerateSubdividedIcosahedron(int degree) {
            var icosahedron = GenerateIcosahedron();

            var nodes = icosahedron.Nodes.Select(t => new Node { Position = t.Position }).ToList();

            var edges = new List<Edge>();
            foreach (var edge in icosahedron.Edges) {
                var n0 = edge.Nodes[0];
                var n1 = edge.Nodes[1];
                var p0 = icosahedron.Nodes[n0].Position;
                var p1 = icosahedron.Nodes[n1].Position;
                nodes[edge.Nodes[0]].Edges.Add(edges.Count);
                var priorNodeIndex = edge.Nodes[0];
                for (int s = 1; s < degree; s++) {
                    var edgeIndex = edges.Count;
                    var nodeIndex = nodes.Count;
                    edge.SubdividedEdges.Add(edgeIndex);
                    edge.SubdividedNodes.Add(nodeIndex);
                    edges.Add(new Edge { Nodes = new List<int> { priorNodeIndex, nodeIndex } });
                    priorNodeIndex = nodeIndex;
                    nodes.Add(new Node { Position = VectorExtensions.Slerp(p0, p1, s / (float)degree), Edges = new List<int> { edgeIndex, edgeIndex + 1 } });
                }
                edge.SubdividedEdges.Add(edges.Count);
                nodes[edge.Nodes[1]].Edges.Add(edges.Count);
                edges.Add(new Edge { Nodes = new List<int> { priorNodeIndex, edge.Nodes[1] } });
            }
            var faces = new List<Face>();
            foreach (var face in icosahedron.Faces) {
                var edge0 = icosahedron.Edges[face.Edges[0]];
                var edge1 = icosahedron.Edges[face.Edges[1]];
                var edge2 = icosahedron.Edges[face.Edges[2]];

                IndexDelegate getEdgeNode0 = (k, f) => (f.Nodes[0] == edge0.Nodes[0]) ? edge0.SubdividedNodes[k] : edge0.SubdividedNodes[degree - 2 - k];
                IndexDelegate getEdgeNode1 = (k, f) => (f.Nodes[1] == edge1.Nodes[0]) ? edge1.SubdividedNodes[k] : edge1.SubdividedNodes[degree - 2 - k];
                IndexDelegate getEdgeNode2 = (k, f) => (f.Nodes[0] == edge2.Nodes[0]) ? edge2.SubdividedNodes[k] : edge2.SubdividedNodes[degree - 2 - k];

                var faceNodes = new List<int> { face.Nodes[0] };
                for (int j = 0; j < edge0.SubdividedNodes.Count; j++) {
                    faceNodes.Add(getEdgeNode0(j, face));
                }
                faceNodes.Add(face.Nodes[1]);

                for (int s = 1; s < degree; s++) {
                    faceNodes.Add(getEdgeNode2(s - 1, face));
                    var p0 = nodes[getEdgeNode2(s - 1, face)].Position;
                    var p1 = nodes[getEdgeNode1(s - 1, face)].Position;
                    for (int t = 1; t < degree - s; t++) {
                        faceNodes.Add(nodes.Count);
                        nodes.Add(new Node { Position = VectorExtensions.Slerp(p0, p1, t / (float)(degree - s)) });
                    }
                    faceNodes.Add(getEdgeNode1(s - 1, face));
                }
                faceNodes.Add(face.Nodes[2]);

                IndexDelegate getEdgeEdge0 = (k, f) => f.Nodes[0] == edge0.Nodes[0] ? edge0.SubdividedEdges[k] : edge0.SubdividedEdges[degree - 1 - k];
                IndexDelegate getEdgeEdge1 = (k, f) => f.Nodes[1] == edge1.Nodes[0] ? edge1.SubdividedEdges[k] : edge1.SubdividedEdges[degree - 1 - k];
                IndexDelegate getEdgeEdge2 = (k, f) => f.Nodes[0] == edge2.Nodes[0] ? edge2.SubdividedEdges[k] : edge2.SubdividedEdges[degree - 1 - k];

                var faceEdges0 = new List<int>();
                for (int j = 0; j < degree; j++) {
                    faceEdges0.Add(getEdgeEdge0(j, face));
                }
                var nodeIndex = degree + 1;
                for (int s = 1; s < degree; s++) {
                    for (int t = 0; t < degree - s; t++) {
                        faceEdges0.Add(edges.Count);
                        var edge = new Edge { Nodes = new List<int> { faceNodes[nodeIndex], faceNodes[nodeIndex + 1] } };
                        nodes[edge.Nodes[0]].Edges.Add(edges.Count);
                        nodes[edge.Nodes[1]].Edges.Add(edges.Count);
                        edges.Add(edge);
                        ++nodeIndex;
                    }
                    ++nodeIndex;
                }

                var faceEdges1 = new List<int>();
                nodeIndex = 1;
                for (var s = 0; s < degree; s++) {
                    for (var t = 1; t < degree - s; t++) {
                        faceEdges1.Add(edges.Count);
                        var edge = new Edge { Nodes = new List<int> { faceNodes[nodeIndex], faceNodes[nodeIndex + degree - s] } };
                        nodes[edge.Nodes[0]].Edges.Add(edges.Count);
                        nodes[edge.Nodes[1]].Edges.Add(edges.Count);
                        edges.Add(edge);
                        ++nodeIndex;
                    }
                    faceEdges1.Add(getEdgeEdge1(s, face));
                    nodeIndex += 2;
                }

                var faceEdges2 = new List<int>();
                nodeIndex = 1;
                for (var s = 0; s < degree; s++) {
                    faceEdges2.Add(getEdgeEdge2(s, face));
                    for (var t = 1; t < degree - s; t++) {
                        faceEdges2.Add(edges.Count);
                        var edge = new Edge { Nodes = new List<int> { faceNodes[nodeIndex], faceNodes[nodeIndex + degree - s + 1] } };
                        nodes[edge.Nodes[0]].Edges.Add(edges.Count);
                        nodes[edge.Nodes[1]].Edges.Add(edges.Count);
                        edges.Add(edge);
                        ++nodeIndex;
                    }
                    nodeIndex += 2;
                }

                nodeIndex = 0;
                var edgeIndex = 0;

                for (var s = 0; s < degree; s++) {
                    for (var t = 1; t < degree - s + 1; t++) {
                        var subFace = new Face {
                            Nodes = new List<int> { faceNodes[nodeIndex], faceNodes[nodeIndex + 1], faceNodes[nodeIndex + degree - s + 1] },
                            Edges = new List<int> { faceEdges0[edgeIndex], faceEdges1[edgeIndex], faceEdges2[edgeIndex] }
                        };
                        nodes[subFace.Nodes[0]].Faces.Add(faces.Count);
                        nodes[subFace.Nodes[1]].Faces.Add(faces.Count);
                        nodes[subFace.Nodes[2]].Faces.Add(faces.Count);
                        edges[subFace.Edges[0]].Faces.Add(faces.Count);
                        edges[subFace.Edges[1]].Faces.Add(faces.Count);
                        edges[subFace.Edges[2]].Faces.Add(faces.Count);
                        faces.Add(subFace);
                        ++nodeIndex;
                        ++edgeIndex;
                    }
                    ++nodeIndex;
                }

                nodeIndex = 1;
                edgeIndex = 0;
                for (var s = 1; s < degree; s++) {
                    for (var t = 1; t < degree - s + 1; t++) {
                        var subFace = new Face {
                            Nodes = new List<int> { faceNodes[nodeIndex], faceNodes[nodeIndex + degree - s + 2], faceNodes[nodeIndex + degree - s + 1] },
                            Edges = new List<int> { faceEdges2[edgeIndex + 1], faceEdges0[edgeIndex + degree - s + 1], faceEdges1[edgeIndex] }
                        };
                        nodes[subFace.Nodes[0]].Faces.Add(faces.Count);
                        nodes[subFace.Nodes[1]].Faces.Add(faces.Count);
                        nodes[subFace.Nodes[2]].Faces.Add(faces.Count);
                        edges[subFace.Edges[0]].Faces.Add(faces.Count);
                        edges[subFace.Edges[1]].Faces.Add(faces.Count);
                        edges[subFace.Edges[2]].Faces.Add(faces.Count);
                        faces.Add(subFace);
                        ++nodeIndex;
                        ++edgeIndex;
                    }
                    nodeIndex += 2;
                    edgeIndex++;
                }
            }
            return new Mesh { Nodes = nodes, Edges = edges, Faces = faces };
        }

        private delegate int IndexDelegate(int k, Face f);

        private static Mesh GenerateIcosahedron() {
            var phi = (1.0 + Math.Sqrt(5.0)) / 2.0;
            var du = (float)(1.0 / Math.Sqrt(phi * phi + 1.0));
            var dv = (float)(phi * du);

            var nodes = new List<Node> {
                new Node {Position = new Vector3(0, dv, du)},
                new Node{Position = new Vector3(0, dv, -du)},
                new Node{Position = new Vector3(0, -dv, du)},
                new Node{Position = new Vector3(0, -dv, -du)},
                new Node{Position = new Vector3(du, 0, dv)},
                new Node{Position = new Vector3(-du, 0, dv)},
                new Node{Position = new Vector3(du, 0, -dv)},
                new Node{Position = new Vector3(-du, 0, -dv)},
                new Node{Position = new Vector3(dv, du, 0)},
                new Node{Position = new Vector3(dv, -du, 0)},
                new Node{Position = new Vector3(-dv, du, 0)},
                new Node{Position = new Vector3(-dv, -du, 0)},
            };
            var edges = new List<Edge> {
                new Edge{Nodes = new List<int>{0,1}},
                new Edge{Nodes = new List<int>{0,4}},
                new Edge{Nodes = new List<int>{0,5}},
                new Edge{Nodes = new List<int>{0,8}},
                new Edge{Nodes = new List<int>{0,10}},
                new Edge{Nodes = new List<int>{1,6}},
                new Edge{Nodes = new List<int>{1,7}},
                new Edge{Nodes = new List<int>{1,8}},
                new Edge{Nodes = new List<int>{1,10}},
                new Edge{Nodes = new List<int>{2,3}},
                new Edge{Nodes = new List<int>{2,4}},
                new Edge{Nodes = new List<int>{2,5}},
                new Edge{Nodes = new List<int>{2,9}},
                new Edge{Nodes = new List<int>{2,11}},
                new Edge{Nodes = new List<int>{3,6}},
                new Edge{Nodes = new List<int>{3,7}},
                new Edge{Nodes = new List<int>{3,9}},
                new Edge{Nodes = new List<int>{3,11}},
                new Edge{Nodes = new List<int>{4,5}},
                new Edge{Nodes = new List<int>{4,8}},
                new Edge{Nodes = new List<int>{4,9}},
                new Edge{Nodes = new List<int>{5,10}},
                new Edge{Nodes = new List<int>{5,11}},
                new Edge{Nodes = new List<int>{6,7}},
                new Edge{Nodes = new List<int>{6,8}},
                new Edge{Nodes = new List<int>{6,9}},
                new Edge{Nodes = new List<int>{7,10}},
                new Edge{Nodes = new List<int>{7,11}},
                new Edge{Nodes = new List<int>{8,9}},
                new Edge{Nodes = new List<int>{10,11}},
            };
            var faces = new List<Face> {
                new Face {Nodes = new List<int>{0,1,8}, Edges = new List<int>{0,7,3}},
                new Face {Nodes = new List<int>{0,4,5}, Edges = new List<int>{1,18,2}},
                new Face {Nodes = new List<int>{0,5,10}, Edges = new List<int>{2,21,4}},
                new Face {Nodes = new List<int>{0,8,4}, Edges = new List<int>{3,19,1}},
                new Face {Nodes = new List<int>{0,10,1}, Edges = new List<int>{4,8,0}},

                new Face {Nodes = new List<int>{1,6,8}, Edges = new List<int>{5,24,7}},
                new Face {Nodes = new List<int>{1,7,6}, Edges = new List<int>{6,23,5}},
                new Face {Nodes = new List<int>{1,10,7}, Edges = new List<int>{8,26,6}},

                new Face {Nodes = new List<int>{2,3,11}, Edges = new List<int>{9,17,13}},
                new Face {Nodes = new List<int>{2,4,9}, Edges = new List<int>{10,20,12}},
                new Face {Nodes = new List<int>{2,5,4}, Edges = new List<int>{11,18,10}},
                new Face {Nodes = new List<int>{2,9,3}, Edges = new List<int>{12,16,9}},
                new Face {Nodes = new List<int>{2,11,5}, Edges = new List<int>{13,22,11}},

                new Face {Nodes = new List<int>{3,6,7}, Edges = new List<int>{14,23,15}},
                new Face {Nodes = new List<int>{3,7,11}, Edges = new List<int>{15,27,17}},
                new Face {Nodes = new List<int>{3,9,6}, Edges = new List<int>{16,25,14}},

                new Face {Nodes = new List<int>{4,8,9}, Edges = new List<int>{19,28,20}},
                new Face {Nodes = new List<int>{5,11,10}, Edges = new List<int>{22,29,21}},
                new Face {Nodes = new List<int>{6,9,8}, Edges = new List<int>{25,28,24}},
                new Face {Nodes = new List<int>{7,10,11}, Edges = new List<int>{26,29,27}},
            };

            for (var i = 0; i < edges.Count; i++) {
                for (var j = 0; j < edges[i].Nodes.Count; j++) {
                    nodes[j].Edges.Add(i);
                }
            }
            for (var i = 0; i < faces.Count; i++) {
                for (var j = 0; j < faces[i].Nodes.Count; j++) {
                    nodes[j].Faces.Add(i);
                }
            }
            for (var i = 0; i < faces.Count; i++) {
                for (var j = 0; j < faces[i].Edges.Count; j++) {
                    edges[j].Faces.Add(i);
                }
            }
            return new Mesh {
                Nodes = nodes,
                Edges = edges,
                Faces = faces
            };
        }
    }

    public class Mesh {
        public List<Node> Nodes { get; set; }
        public List<Edge> Edges { get; set; }
        public List<Face> Faces { get; set; }
    }

    public class Node {
        public Vector3 Position { get; set; }
        public List<int> Edges = new List<int>();
        public readonly List<int> Faces = new List<int>();
    }

    public class Edge {
        public List<int> Nodes = new List<int>();
        public readonly List<int> Faces = new List<int>();

        public readonly List<int> SubdividedNodes = new List<int>();
        public readonly List<int> SubdividedEdges = new List<int>();
    }

    public class Face {
        public List<int> Nodes = new List<int>();
        public List<int> Edges = new List<int>();
    }

    public static class VectorExtensions {
        public static Vector3 Slerp(Vector3 v0, Vector3 v1, float t) {
            var omega = Math.Acos(Vector3.Dot(v0, v1));
            return (v0 * (float)Math.Sin((1 - t) * omega) + v1 * (float)(Math.Sin(t * omega))) / (float)Math.Sin(omega);
        }
    }
}
