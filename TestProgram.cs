using System;
using System.Collections.Generic;
using DStarLite;

namespace DStarLiteTest
{
    class TestProgram
    {
        static void Main(string[] args)
        {
            Test test = new Test();
            DStarLite.DStarLite.RunDStarLite(0, 1, 8, 6, test);
            Console.WriteLine("Press enter to close...");
            Console.ReadLine();
        }
    }

    class Test : DStarLite.DStarLiteEnvironment
    {
        int time = 0;
        int px = 0;
        int py = 1;

        public void MoveTo(Coordinates s)
        {
            time++;
            px = s.x;
            py = s.y;
            Console.WriteLine("Move " + time + ": " + px + " " + py);
        }

        public LinkedList<Coordinates> GetObstaclesInVision()
        {
            LinkedList<Coordinates> l = new LinkedList<Coordinates>();
            if (time == 1)
            {
                l.AddFirst(new Coordinates(1, 2));
                l.AddFirst(new Coordinates(1, 3));
                l.AddFirst(new Coordinates(0, 3));
            }
            if (time == 2)
            {
                l.AddFirst(new Coordinates(3, 1));
                l.AddFirst(new Coordinates(3, 2));
            }
            if (py == 3)
            {
                l.AddFirst(new Coordinates(2, 4));
                l.AddFirst(new Coordinates(5, 4));
            }
            if (px == 4)
            {
                l.AddFirst(new Coordinates(5, 1));
                l.AddFirst(new Coordinates(5, 2));
            }
            if (px == 5)
            {
                l.AddFirst(new Coordinates(7, 1));
            }
            if (px == 6)
            {
                l.AddFirst(new Coordinates(8, 2));
            }
            if (px == 7)
            {
                l.AddFirst(new Coordinates(8, 1));
            }
            if (py == 5)
            {
                l.AddFirst(new Coordinates(7, 5));
                l.AddFirst(new Coordinates(5, 6));
                l.AddFirst(new Coordinates(4, 7));
            }
            if (py == 3)
            {
                l.AddFirst(new Coordinates(8, 3));
            }
            if (px == 4 || py == 3)
            {
                l.AddFirst(new Coordinates(4, 4));
            }
            if (py == 3 || px == 6)
            {
                l.AddFirst(new Coordinates(7, 4));
            }
            if (px < 3 && py > 4)
            {
                l.AddFirst(new Coordinates(1, 6));
            }
            if ((px == 3 && py != 0) || py == 5)
            {
                l.AddFirst(new Coordinates(3, 6));
            }
            return l;
        }
    }
}
