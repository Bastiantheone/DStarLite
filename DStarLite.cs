using System;
using System.Collections.Generic;

namespace DStarLite
{
    public class DStarLite
    {
        static Heap U;
        static State[,] S;
        static double km;
        static State sgoal;
        static State sstart;

        // sx and sy are the start coordinates and gx and gy are the goal coordinates
        public static void RunDStarLite(int sx, int sy, int gx, int gy, DStarLiteEnvironment env)
        {
            sstart = new State();
            sstart.x = sx;
            sstart.y = sy;
            sgoal = new State();
            sgoal.x = gx;
            sgoal.y = gy;
            State slast = sstart;
            Initialize();
            ComputeShortestPath();
            while (!sstart.Equals(sgoal))
            {
                // if(sstart.g.isInfinity) then there is no known path
                sstart = MinSuccState(sstart);
                env.MoveTo(new Coordinates(sstart.x, sstart.y));
                LinkedList<Coordinates> obstacleCoord = env.GetObstaclesInVision();
                double oldkm = km;
                State oldslast = slast;
                km += Heuristic(sstart, slast);
                slast = sstart;
                bool change = false;
                foreach (Coordinates c in obstacleCoord)
                {
                    State s = S[c.x, c.y];
                    if (s.obstacle) continue;// is already known
                    change = true;
                    s.obstacle = true;
                    foreach (State p in s.GetPred())
                    {
                        UpdateVertex(p);
                    }
                }
                if (!change)
                {
                    km = oldkm;
                    slast = oldslast;
                }
                ComputeShortestPath();
            }
        }

        // calculates the key
        /*
        Priority of a vertex = key
        Key – vector with 2 components
            k(s) = [ k1(s);  k2(s) ]

        k1(s) = min(g(s), rhs(s)) + h(s, sstart) + km
        k2(s) = min(g(s), rhs(s)) 
        */
        static K CalculateKey(State s)
        {
            return new K(min(s.g, s.rhs) + Heuristic(s, sstart) + km, min(s.g, s.rhs));
        }

        static double Heuristic(State a, State b)
        {
            return Math.Abs(a.x - b.x) + Math.Abs(a.y - b.y);
        }

        // runs on a 100*100 plane
        static void Initialize()
        {
            U = new Heap(10000);
            S = new State[100, 100];
            km = 0;
            for (int i = 0; i < S.GetLength(0); i++)
            {
                for (int j = 0; j < S.GetLength(1); j++)
                {
                    S[i, j] = new State();
                    S[i, j].x = i;
                    S[i, j].y = j;
                    S[i, j].g = Double.PositiveInfinity;
                    S[i, j].rhs = Double.PositiveInfinity;
                }
            }
            sgoal = S[sgoal.x, sgoal.y];
            sstart = S[sstart.x, sstart.y];
            sgoal.rhs = 0;
            U.Insert(sgoal, CalculateKey(sgoal));
        }

        static void UpdateVertex(State u)
        {
            if (!u.Equals(sgoal))
            {
                u.rhs = MinSucc(u);
            }
            if (U.Contains(u))
            {
                U.Remove(u);
            }
            if (u.g != u.rhs)
            {
                U.Insert(u, CalculateKey(u));
            }
        }

        static State MinSuccState(State u)
        {
            double min = Double.PositiveInfinity;
            State n = null;
            foreach (State s in u.GetSucc())
            {
                double val = 1 + s.g;
                if (val <= min && !s.obstacle)
                {
                    min = val;
                    n = s;
                }
            }
            return n;
        }

        // finds the succesor s' with the min (c(u,s')+g(s'))
        // where cost from u to s' is 1 and returns the value
        static double MinSucc(State u)
        {
            double min = Double.PositiveInfinity;
            foreach (State s in u.GetSucc())
            {
                double val = 1 + s.g;
                if (val < min && !s.obstacle) min = val;
            }
            return min;
        }

        static void ComputeShortestPath()
        {
            while (U.TopKey().CompareTo(CalculateKey(sstart)) < 0 || sstart.rhs != sstart.g)
            {
                K kold = U.TopKey();
                State u = U.Pop();
                if (u == null) break;
                if (kold.CompareTo(CalculateKey(u)) < 0)
                {
                    U.Insert(u, CalculateKey(u));
                }
                else if (u.g > u.rhs)
                {
                    u.g = u.rhs;
                    foreach (State s in u.GetPred())
                    {
                        UpdateVertex(s);
                    }
                }
                else
                {
                    u.g = Double.PositiveInfinity;
                    UpdateVertex(u);
                    foreach (State s in u.GetPred())
                    {
                        UpdateVertex(s);
                    }
                }
            }
        }

        static double min(double a, double b)
        {
            if (b < a) return b;
            return a;
        }

        class State
        {
            public int x;
            public int y;
            public double g;
            public double rhs;
            public bool obstacle;

            public bool Equals(State that)
            {
                if (this.x == that.x && this.y == that.y) return true;
                return false;
            }

            public LinkedList<State> GetSucc()
            {
                LinkedList<State> s = new LinkedList<State>();
                // add succesors in counter clockwise order
                if (x + 1 < S.GetLength(0)) s.AddFirst(S[x + 1, y]);
                if (y + 1 < S.GetLength(1)) s.AddFirst(S[x, y + 1]);
                if (x - 1 >= 0) s.AddFirst(S[x - 1, y]);
                if (y - 1 >= 0) s.AddFirst(S[x, y - 1]);
                return s;
            }

            public LinkedList<State> GetPred()
            {
                LinkedList<State> s = new LinkedList<State>();
                State tempState;
                // add predecessors in counter clockwise order if they are not an obstacle
                if (x + 1 < S.GetLength(0))
                {
                    tempState = S[x + 1, y];
                    if (!tempState.obstacle) s.AddFirst(tempState);
                }
                if (y + 1 < S.GetLength(1))
                {
                    tempState = S[x, y + 1];
                    if (!tempState.obstacle) s.AddFirst(tempState);
                }
                if (x - 1 >= 0)
                {
                    tempState = S[x - 1, y];
                    if (!tempState.obstacle) s.AddFirst(tempState);
                }
                if (y - 1 >= 0)
                {
                    tempState = S[x, y - 1];
                    if (!tempState.obstacle) s.AddFirst(tempState);
                }
                return s;
            }
        }

        class K
        {
            public double k1;
            public double k2;

            public K(double K1, double K2)
            {
                k1 = K1;
                k2 = K2;
            }

            public int CompareTo(K that)
            {
                if (this.k1 < that.k1) return -1;
                else if (this.k1 > that.k1) return 1;
                if (this.k2 > that.k2) return 1;
                else if (this.k2 < that.k2) return -1;
                return 0;
            }
        }

        class HeapElement
        {
            public State s;
            public K k;

            public HeapElement(State state, K key)
            {
                s = state;
                k = key;
            }
        }

        // min heap
        class Heap
        {
            private int n;
            private HeapElement[] heap;
            private Dictionary<State, int> hash;

            public Heap(int cap)
            {
                n = 0;
                heap = new HeapElement[cap];
                hash = new Dictionary<State, int>();
            }

            public K TopKey()
            {
                if (n == 0) return new K(Double.PositiveInfinity, Double.PositiveInfinity);
                return heap[1].k;
            }

            public State Pop()
            {
                if (n == 0) return null;
                State s = heap[1].s;
                heap[1] = heap[n];
                hash[heap[1].s] = 1;
                hash[s] = 0;
                n--;
                moveDown(1);
                return s;
            }

            public void Insert(State s, K k)
            {
                HeapElement e = new HeapElement(s, k);
                n++;
                hash[s] = n;
                if (n == heap.Length) increaseCap();
                heap[n] = e;
                moveUp(n);
            }

            public void Update(State s, K k)
            {
                int i = hash[s];
                if (i == 0) return;
                K kold = heap[i].k;
                heap[i].k = k;
                if (kold.CompareTo(k) < 0)
                {
                    moveDown(i);
                }
                else
                {
                    moveUp(i);
                }
            }

            public void Remove(State s)
            {
                int i = hash[s];
                if (i == 0) return;
                hash[s] = 0;
                heap[i] = heap[n];
                hash[heap[i].s] = i;
                n--;
                moveDown(i);
            }

            public bool Contains(State s)
            {
                int i;
                if (!hash.TryGetValue(s, out i))
                {
                    return false;
                }
                return i != 0;
            }

            private void moveDown(int i)
            {
                int childL = i * 2;
                if (childL > n) return;
                int childR = i * 2 + 1;
                int smallerChild;
                if (childR > n)
                {
                    smallerChild = childL;
                }
                else if (heap[childL].k.CompareTo(heap[childR].k) < 0)
                {
                    smallerChild = childL;
                }
                else
                {
                    smallerChild = childR;
                }
                if (heap[i].k.CompareTo(heap[smallerChild].k) > 0)
                {
                    swap(i, smallerChild);
                    moveDown(smallerChild);
                }
            }

            private void moveUp(int i)
            {
                if (i == 1) return;
                int parent = i / 2;
                if (heap[parent].k.CompareTo(heap[i].k) > 0)
                {
                    swap(parent, i);
                    moveUp(parent);
                }
            }

            private void swap(int i, int j)
            {
                HeapElement temp = heap[i];
                heap[i] = heap[j];
                hash[heap[j].s] = i;
                heap[j] = temp;
                hash[temp.s] = j;
            }

            private void increaseCap()
            {
                Array.Resize<HeapElement>(ref heap, heap.Length * 2);
            }
        }
    }

    public class Coordinates
    {
        public int x;
        public int y;

        public Coordinates(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }

    public interface DStarLiteEnvironment
    {
        void MoveTo(Coordinates s);
        LinkedList<Coordinates> GetObstaclesInVision();
    }

}
