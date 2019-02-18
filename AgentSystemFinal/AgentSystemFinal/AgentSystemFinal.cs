using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// <Custom using>
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
// </Custom using>


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public class Script_Instance : GH_ScriptInstance
{
    #region Utility functions
    /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
    /// <param name="text">String to print.</param>
    private void Print(string text) { __out.Add(text); }
    /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
    /// <param name="format">String format.</param>
    /// <param name="args">Formatting parameters.</param>
    private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
    /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
    /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
    /// <param name="obj">Object instance to parse.</param>
    private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
    #endregion

    #region Members
    /// <summary>Gets the current Rhino document.</summary>
    private RhinoDoc RhinoDocument;
    /// <summary>Gets the Grasshopper document that owns this script.</summary>
    private GH_Document GrasshopperDocument;
    /// <summary>Gets the Grasshopper script component that owns this script.</summary>
    private IGH_Component Component;
    /// <summary>
    /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
    /// Any subsequent call within the same solution will increment the Iteration count.
    /// </summary>
    private int Iteration;
    #endregion

    /// <summary>
    /// This procedure contains the user code. Input parameters are provided as regular arguments, 
    /// Output parameters as ref arguments. You don't have to assign output parameters, 
    /// they will have a default value.
    /// </summary>
    private void RunScript(bool reset, bool go, Mesh MScalar, Mesh MVector, double vectorStrength, double scalarStrength, double meshSeekRad, double meshStrength, List<Point3d> agPos, List<Vector3d> agVel, double agMaxSpeed, double futPosMult, double trailSeekRad, double agAngVis, double coheStrength, double aligStrength, double sepaStrength, double sepaRadius, int trailFreq, Rectangle3d BodyRectangle, ref object Ap, ref object Av, ref object neigh, ref object Tr, ref object BPl)
    {
        // <Custom code>

        // output geometry initialization
        OutputGeom geomOut;

        // if base inputs are null return and do nothing
        if (MScalar == null || MVector == null || agPos == null || agVel == null) return;

        // initialize system on reset (or at first component run)
        if (reset || AgSys == null)
        {
            // initializing Mesh related variables
            MEnvironment = MScalar;                                 // Here you can use either one between MScalar and MVector
            MeshRTree = RTreeFromMesh(MScalar);                     // initialize Mesh RTree
            TensorField = TensorFieldFromMeshes(MScalar, MVector);    // initialize Tensor Point Field

            // initializing Trail related variables
            TrailPoints = new List<TrailPoint>();                   // List of all trail points
            TrailsRTree = new RTree();                              // initialize trails RTree
            TrailPtCount = 0;                                       // total count of trail points (as index to insert points in trails RTree)

            // initializing Body Planes related variables
            BodyPlanes = new List<Plane>();                         // List of all Body Planes
            BodyPlanesRTree = new RTree();                          // initialize Body Planes RTree
            bodyPlCount = 0;                                        // total count of Body Planes (as index to insert points in Body Planes RTree)

            // initializing Agent System and global variables
            AgSys = new AgentSystem(agPos, agVel);
            count = 0;

            debug = ""; // debug string
            /*
             * HOW TO USE DEBUG STRING
             * Since it is not possible to compile directly this code in VS, the trick is to define a debug string and add
             * checks and flags messages (or variable values) to it. Then uncheck the Print(debug) instruction at the bottom of this method.
             * 
             * Examples:
             * 
             * . check if a part of the code is executed:
             * debug += "I have been executed\n"; // the \n is an excape sequence equivalent to the Return button on the keyboard
             * 
             * . store a variable:
             * debug += Agent.position.ToString() + "\n"; // remember to convert to string (and add the new line escape sequence)
             */
        }

        if (go)
        {
            // update parameters 
            AgSys.VectorFieldStrength = vectorStrength;
            AgSys.ScalarFieldStrength = scalarStrength;
            AgSys.MeshSeekRadius = meshSeekRad;
            AgSys.MeshStrength = meshStrength;
            AgSys.MaxSpeed = agMaxSpeed;
            AgSys.futPosMultiplier = futPosMult;
            AgSys.TrailSeekRadius = trailSeekRad;
            AgSys.AngVis = agAngVis;
            AgSys.CohesionStrength = coheStrength;
            AgSys.AlignmentStrength = aligStrength;
            AgSys.SeparationStrength = sepaStrength;
            AgSys.SeparationRadius = sepaRadius;
            AgSys.TrailFrequency = trailFreq;
            AgentBodyWidth = BodyRectangle.X[1];
            AgentBodyHeight = BodyRectangle.Y[1];

            // update system
            AgSys.UpdateRTree();

            // update counter
            count++;

            // run component
            Component.ExpireSolution(true);
        }

        // extract geometries for output
        geomOut = AgSys.GetAllOut();

        // uncomment for DEBUG ONLY
        //geomOut.neighOut = AgSys.GetNeighbours(); // 1 of 2 - uncomment to output neighbour points for agents (debug mode)
        //Print(debug); // Prints debug messages stored in debug string

        Ap = geomOut.ptsOut;
        Av = geomOut.vecsOut;
        //neigh = geomOut.neighOut; // 2 of 2 - uncomment to output neighbour points for agents (debug mode)
        Tr = geomOut.trailsOut;
        BPl = BodyPlanes;

        // </Custom code>
    }

    // <Custom additional code> 

    // global variables
    public AgentSystem AgSys;                   // the agent system
    public static TensorPoint[] TensorField;    // tensor point array
    public static Mesh MEnvironment;            // the environment mesh
    public static RTree MeshRTree;              // RTree of mesh vertices

    public static List<TrailPoint> TrailPoints; // List of all TrailPoints
    public static RTree TrailsRTree;            // RTree of trail points
    public static int TrailPtCount;             // counter for Trail Points RTree

    public static List<Plane> BodyPlanes;       // List of all body planes
    public static RTree BodyPlanesRTree;        // Rree of body planes
    public static int bodyPlCount;              // counter for body planes RTree

    public static int count;                    // global counter for frequency
    public static double AgentBodyWidth;        // Agent Body Width
    public static double AgentBodyHeight;       // Agent Body Height

    public static String debug;                // the debug String

    // classes

    /// <summary>
    /// This class is the global Agent System simulation
    /// </summary>
    public class AgentSystem
    {
        public List<Agent> Agents;
        public List<Polyline> AgentsTrails;

        public double VectorFieldStrength;
        public double ScalarFieldStrength;
        public double MeshSeekRadius;
        public double MeshStrength;

        public double MaxSpeed;
        public double futPosMultiplier;
        public double TrailSeekRadius;
        public double AngVis;

        public double CohesionStrength;
        public double AlignmentStrength;
        public double SeparationStrength;
        public double SeparationRadius;

        public int TrailFrequency;


        public AgentSystem(List<Point3d> P, List<Vector3d> V)
        {
            Agents = new List<Agent>();

            for (int i = 0; i < P.Count; i++)
            {
                Agent ag = new Agent(P[i], V[i]);
                ag.Agents = this;

                Agents.Add(ag);
            }

        }


        /// <summary>
        /// This method is used with RTree for fast neighbour points search.
        /// </summary>
        public void UpdateRTree()
        {
            // debug string
            debug = "";

            // Flock with trails (OLD method)
            //List<Polyline> AllTrails = new List<Polyline>();
            //foreach (Agent ag in Agents)
            //    AllTrails.Add(ag.trail);

            foreach (Agent ag in Agents)
            {
                // if agent is not alive do not update and go to the next
                if (!ag.isAlive) continue;

                // clear neighbours list
                ag.neighTens.Clear();

                List<TrailPoint> neighTrails = new List<TrailPoint>();

                // Eventhandler callback function for Mesh RTree search
                EventHandler<RTreeEventArgs> MeshRTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    //Vector3d dir = Menv.Vertices[args.Id] - (ag.position + ag.velocity * futPosMultiplier);
                    //if (Vector3d.VectorAngle(ag.velocity, dir) < AngVis)
                    //{
                    //ag.neighbours.Add(MVector.Vertices[args.Id]);
                    ag.neighTens.Add(TensorField[args.Id]);
                    //}
                };

                // Eventhandler callback function for Trails RTree search
                EventHandler<RTreeEventArgs> TrailsRTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    // if trail point found is within angle of vision include in trail points neighbours
                    Vector3d dir = TrailPoints[args.Id].position - (ag.position + ag.velocity * futPosMultiplier);
                    if (Vector3d.VectorAngle(ag.velocity, dir) < AngVis)
                    {
                        neighTrails.Add(TrailPoints[args.Id]);
                    }
                };

                //                   Sphere(                     future position projected on Mesh                 ,   radius      ), callback function
                MeshRTree.Search(new Sphere(MEnvironment.ClosestPoint(ag.position + ag.velocity * futPosMultiplier), MeshSeekRadius), MeshRTreeCallback);

                //                   Sphere(               future position                ,   radius          ), callback function
                TrailsRTree.Search(new Sphere(ag.position + ag.velocity * futPosMultiplier, TrailSeekRadius), TrailsRTreeCallback);

                ag.ResetDesired();

                // the search neighbours functions for the Mesh points and TensorPoints
                // have been combined as:
                ag.SeekCombined(MeshStrength, ScalarFieldStrength, VectorFieldStrength);

                // Flock with trails
                if (neighTrails.Count > 0)
                    ag.FlockTrails(neighTrails);

                // Flock with trails (OLD METHOD)
                //ag.FlockTrailsClosestPt(AllTrails);


                // calls custom behavior (go to function implementation to write your code)
                ag.CustomBehavior();
            }

            // Update Agents Velocity and Positions (parallelized if n. of agents > 500)
            if (Agents.Count > 500)
            {
                Parallel.ForEach(Agents, ag =>
                {
                    ag.UpdateVelocityAndPosition();
                });
            }
            else
                foreach (Agent ag in Agents)
                    ag.UpdateVelocityAndPosition();
        }

        /// <summary>
        /// This method extracts output geometries and data in output separate arrays.
        /// </summary>
        public void GetAllOut(out GH_Point[] pts, out GH_Vector[] vecs, out Polyline[] trailsOut)
        {
            pts = new GH_Point[Agents.Count];
            vecs = new GH_Vector[Agents.Count];
            trailsOut = new Polyline[Agents.Count];

            for (int i = 0; i < Agents.Count; i++)
            {
                pts[i] = new GH_Point(Agents[i].position);
                vecs[i] = new GH_Vector(Agents[i].velocity);
                trailsOut[i] = Agents[i].trail;
            }
        }

        /// <summary>
        /// This method extracts output geometries and data in a dedicated class.
        /// </summary>
        public OutputGeom GetAllOut()
        {
            OutputGeom gOut = new OutputGeom(Agents.Count);


            //Parallel.For(0, Agents.Count, i =>
            for (int i = 0; i < Agents.Count; i++)
            {
                gOut.ptsOut[i] = new GH_Point(Agents[i].position);
                gOut.vecsOut[i] = new GH_Vector(Agents[i].velocity);
                gOut.trailsOut[i] = Agents[i].trail;
            }//);

            return gOut;
        }

        /// <summary>
        /// This method gets all Agents neighbours for debug output
        /// </summary>
        /// <returns>Neighbours for all agents as a Data Tree of GH_Point</returns>
        public DataTree<GH_Point> GetNeighbours()
        {
            DataTree<GH_Point> neighOut = new DataTree<GH_Point>();

            for (int i = 0; i < Agents.Count; i++)
            {
                neighOut.AddRange(Agents[i].GetNeighbours(), new GH_Path(i));
            }

            return neighOut;
        }

    }

    /// <summary>
    /// This class is a typical agent capable of Reynolds classic behaviors + customizable ones
    /// </summary>
    public class Agent
    {
        // fields
        public Point3d position;
        public Vector3d velocity;
        public Vector3d desiredVelocity;
        public Polyline trail;
        public AgentSystem Agents;
        public List<Point3d> neighbours;
        public List<TensorPoint> neighTens;
        public bool isAlive; // allows us to stop updating the agent in certain situations - not used yet but checked by AgentSystem.Update()

        // constructor
        public Agent(Point3d position, Vector3d velocity)
        {
            this.position = position;
            this.velocity = velocity;
            desiredVelocity = this.velocity;
            trail = new Polyline { this.position, this.position + this.velocity };

            // trail points added to a list and RTree for fast search
            TrailPoints.Add(new TrailPoint(this.position, this.velocity));
            TrailsRTree.Insert(this.position, TrailPtCount);
            TrailPtCount++;

            // neighbour points lists
            neighbours = new List<Point3d>(); // not used in this simulation
            neighTens = new List<TensorPoint>();
            isAlive = true;
        }

        // methods
        public void ResetDesired()
        {
            desiredVelocity = velocity;
        }

        /// <summary>
        /// This method implements flocking with trails neighbour points
        /// </summary>
        /// <param name="trailNeighbours">the neighbour trail points</param>
        public void FlockTrails(List<TrailPoint> trailNeighbours)
        {
            // ................................... COHESION BEHAVIOUR .....................
            //
            // find neighbours average
            Point3d average = new Point3d();

            foreach (TrailPoint neighbour in trailNeighbours)
                average += neighbour.position;

            average /= trailNeighbours.Count;

            // go there
            Vector3d cohesion = average - position;

            desiredVelocity += cohesion * Agents.CohesionStrength;

            // ................................... ALIGNMENT BEHAVIOUR .....................
            //
            Vector3d alignment = Vector3d.Zero;

            foreach (TrailPoint neighbour in trailNeighbours)
                alignment += neighbour.velocity;

            alignment /= trailNeighbours.Count;

            desiredVelocity += alignment * Agents.AlignmentStrength;

            // ................................... SEPARATION BEHAVIOUR .....................
            //
            Vector3d separation = Vector3d.Zero;

            foreach (TrailPoint trailNeighbour in trailNeighbours)
            {
                double distanceToNeighbour = position.DistanceTo(trailNeighbour.position);
                if (distanceToNeighbour < Agents.SeparationRadius)
                {
                    Vector3d getAway = position - trailNeighbour.position;
                    separation += getAway /= (getAway.Length * distanceToNeighbour);
                }

            }
            // sometimes, separation may result in a vector whose coordinates are NaN (Not a Number - the equivalent of null for int and double)
            // so, we need to check for NaN
            if (!Double.IsNaN(separation.X))
                desiredVelocity += separation * Agents.SeparationStrength;

        }

        /// <summary>
        /// This method implements flocking with the closest point to the closest trail
        /// </summary>
        /// <param name="AllTrails">all Agents trails as a List of Polyline</param>
        public void FlockTrailsClosestPt(List<Polyline> AllTrails)
        {
            // find closest trail point and dir

            double trailParameter;
            Point3d testPoint = new Point3d();
            Point3d nearest = new Point3d();
            Point3d futPos = position + velocity;
            Vector3d nearestDir = new Vector3d();
            double dd;
            double MinD = Double.MaxValue;
            foreach (Polyline tr in AllTrails)
            {
                // if it's not the trail of the current agent....
                if (AllTrails.IndexOf(tr) != Agents.Agents.IndexOf(this))
                {
                    // evaluate closest point and its distance
                    trailParameter = tr.ClosestParameter(futPos);
                    testPoint = tr.PointAt(trailParameter);
                    dd = testPoint.DistanceToSquared(futPos);
                    // if closer that the min distance
                    if (dd < MinD)
                    {
                        MinD = dd;
                        nearest = new Point3d(testPoint);
                        nearestDir = tr.TangentAt(trailParameter);
                    }
                }

            }

            FlockSinglePoint(nearest, nearestDir);
        }

        /// <summary>
        /// Implements flocking with a single point - used by FlockTrailsClosestPt
        /// </summary>
        /// <param name="target"></param>
        /// <param name="targetVel"></param>
        public void FlockSinglePoint(Point3d target, Vector3d targetVel)
        {
            // ................................... COHESION BEHAVIOUR .....................
            //
            // go there
            Vector3d cohesion = target - position;

            desiredVelocity += cohesion * Agents.CohesionStrength;

            // ................................... ALIGNMENT BEHAVIOUR .....................
            //
            Vector3d alignment = Vector3d.Zero;

            desiredVelocity += targetVel * Agents.AlignmentStrength;

            // ................................... SEPARATION BEHAVIOUR .....................
            //
            Vector3d separation = Vector3d.Zero;

            double distanceToNeighbour = position.DistanceTo(target);
            if (distanceToNeighbour < Agents.SeparationRadius)
            {
                Vector3d getAway = position - target;
                separation += getAway /= (getAway.Length * distanceToNeighbour);
            }
            desiredVelocity += separation * Agents.SeparationStrength;
        }

        /// <summary>
        /// Seek behavior with a target Point3d and a given strength
        /// </summary>
        /// <param name="target"></param>
        /// <param name="strength"></param>
        public void SeekPoint(Point3d target, double strength)
        {
            Vector3d seek = target - position;
            seek.Unitize();
            seek *= Agents.MaxSpeed;

            desiredVelocity += seek * strength;
        }

        /// <summary>
        /// Seek the neighbour TensorPoint List aiming for the highest value in scalar field and aligning with vector field
        /// </summary>
        /// <param name="meshStrength">strength with which to seek mesh points</param>
        /// <param name="scalarStrength">strength with which to seek point with highest scalar value</param>
        /// <param name="vectorStrength">strength with which to align with highest value direction</param>
        public void SeekCombined(double meshStrength, double scalarStrength, double vectorStrength)
        {
            // if there are no neighbours return
            if (neighTens.Count <= 0) return;

            double val;
            double MaxVal = -1.0;
            int MaxInd = -1;
            Point3d average = new Point3d();

            // find point with max intensity
            for (int i = 0; i < neighTens.Count; i++)
            {
                average += neighTens[i].position;

                val = neighTens[i].scalar; // brightness is 0-1 and RGB 0-255
                if (val > MaxVal)
                {
                    MaxVal = val;
                    MaxInd = i;
                }
            }

            // mesh contribution
            average /= neighTens.Count;
            SeekPoint(average, meshStrength);

            // field (vector + scalar) contribution
            Vector3d seek = (neighTens[MaxInd].position - position) * scalarStrength + neighTens[MaxInd].vector * vectorStrength;

            desiredVelocity += seek;
        }

        public void CustomBehavior()
        {
            // add your custom behavior here

            // end of custom behavior
        }

        /// <summary>
        /// Updates velocity and position for the agent, trail points and calls body plane deposition
        /// </summary>
        public void UpdateVelocityAndPosition()
        {
            // steering method
            velocity = 0.97 * velocity + 0.03 * desiredVelocity;

            // limit the velocity to maximum speed (0.5)
            if (velocity.Length > Agents.MaxSpeed)
            {
                velocity.Unitize();
                velocity *= Agents.MaxSpeed;
            }

            position += velocity;

            // update trail on given frequency
            if (count % Agents.TrailFrequency == 0)
            {
                // ad trail point to global list and RTree
                TrailPoints.Add(new TrailPoint(position, velocity));
                TrailsRTree.Insert(position, TrailPtCount);
                TrailPtCount++;

                // add trail point to trail polyline
                trail.Add(position);

                // tries to deposit a body plane
                DepositPlane();
            }

        }

        /// <summary>
        /// checks if a body plane can be deposited and if so adds it to the List
        /// </summary>
        public void DepositPlane()
        {
            List<Plane> neighPlanes = new List<Plane>();

            // Eventhandler callback function for Mesh RTree search
            EventHandler<RTreeEventArgs> BodyRTreeCallback = (object sender, RTreeEventArgs args) =>
            {
                neighPlanes.Add(BodyPlanes[args.Id]);
            };

            MeshPoint mp = MEnvironment.ClosestMeshPoint(position, 100);

            //                 Sphere(position projected on Mesh, radius), callback function
            BodyPlanesRTree.Search(new Sphere(mp.Point, 5.0), BodyRTreeCallback);

            Vector3d xAxis = new Vector3d(velocity);

            if (neighPlanes.Count > 0)
            {
                Point3d PtOnPlane;
                Vector3d alignDir = Vector3d.Zero;

                bool deposit = true;
                foreach (Plane neighPlane in neighPlanes)
                {
                    // project candidate plane origin on each neighbour plane and convert into plane coordinates
                    neighPlane.RemapToPlaneSpace(neighPlane.ClosestPoint(mp.Point), out PtOnPlane);

                    // if point is within body rectangle of at least one neighbour exit loop
                    if (Math.Abs(PtOnPlane.X) < AgentBodyWidth && Math.Abs(PtOnPlane.Y) < AgentBodyHeight)
                    {
                        deposit = false;
                        break;
                    }
                    alignDir += neighPlane.XAxis;
                }

                if (!deposit) return;

                alignDir /= neighPlanes.Count;

                xAxis = xAxis * 0.97 + alignDir * 0.03;
            }

            // build body plane
            Vector3d yAxis = Vector3d.CrossProduct(velocity, MEnvironment.NormalAt(mp));
            Plane pl = new Plane(mp.Point, xAxis, yAxis);

            // Add plane to the list of Body Planes
            BodyPlanes.Add(pl);

            // Add origin to RTree and increase count
            BodyPlanesRTree.Insert(pl.Origin, bodyPlCount);
            bodyPlCount++;
        }

        /// <summary>
        /// This function gets Agent neighbours for output
        /// </summary>
        /// <returns>a List of GH_Point</returns>
        public List<GH_Point> GetNeighbours()
        {
            List<GH_Point> neighOut = new List<GH_Point>();

            foreach (TensorPoint n in neighTens)
                neighOut.Add(new GH_Point(n.position));

            return neighOut;
        }

    }

    /// <summary>
    /// This class associates vector and scalar values with a point in 3D space.
    /// </summary>
    public class TensorPoint
    {
        public Point3d position;
        public Vector3d vector;
        public double scalar;

        public TensorPoint(Point3d position, Vector3d vector, double scalar)
        {
            this.position = position;
            this.vector = vector;
            this.scalar = scalar;
        }
    }

    /// <summary>
    /// This class acts as a container for output geometry.
    /// </summary>
    public class OutputGeom
    {

        public GH_Point[] ptsOut;
        public GH_Vector[] vecsOut;
        public Polyline[] trailsOut;
        public DataTree<GH_Point> neighOut;
        private int size;

        public OutputGeom(int size)
        {
            this.size = size;
            ptsOut = new GH_Point[this.size];
            vecsOut = new GH_Vector[this.size];
            trailsOut = new Polyline[this.size];
            neighOut = new DataTree<GH_Point>();
        }
    }

    /// <summary>
    /// This class acts as a container for trail points, storing position and agent velocity at that position.
    /// </summary>
    public class TrailPoint
    {
        public Point3d position;
        public Vector3d velocity;

        public TrailPoint(Point3d position, Vector3d velocity)
        {
            this.position = position;
            this.velocity = velocity;
        }

        // "zero" constructor
        public TrailPoint() : this(new Point3d(), Vector3d.Zero)
        { }
    }

    // utilities functions

    /// <summary>
    /// This method builds an RTree from a Mesh, adding all vertices.
    /// </summary>
    /// <param name="M">An input Mesh</param>
    /// <returns>An RTree containing the positions of the Mesh vertices</returns>
    public RTree RTreeFromMesh(Mesh M)
    {
        // compact creation mode
        return RTree.CreateFromPointArray(M.Vertices.ToPoint3dArray());

        // extensive creation mode (adding points one by one)
        //RTree MPoints = new RTree();
        //for (int i = 0; i < M.Vertices.Count; i++)
        //    MPoints.Insert(M.Vertices[i], i);
        //return MPoints;
    }

    /// <summary>
    /// This method builds a TensorPoint array from two colored Meshes, one TensorPoint for each Mesh vertex.
    /// </summary>
    /// <remarks>
    /// The Mesh colors are converted into 0 to 1 range from brightness for the scalar field and
    /// to a -1.0 to 1.0 range for Vector X, Y, Z coordinate from 0-255 RGB values respectively.
    /// MScalar and MVector must have the same number of vertices (M.Vertices.Count) and indexed identically
    /// The best strategy is to build MScalar and MVector from the same mesh, just assigning different colors:
    /// . a grayscale pattern for MScalar
    /// . an RBG pattern for MVector
    /// </remarks>
    /// <param name="MScalar">Colored Mesh for scalar field</param>
    /// <param name="MVector">Colored Mesh for Vector field</param>
    /// <returns>An array of TensorPoint, its Length equal to the number of vertices in each Mesh and with corresponding indexes.</returns>
    public TensorPoint[] TensorFieldFromMeshes(Mesh MScalar, Mesh MVector)
    {
        TensorPoint[] MeshField = new TensorPoint[MScalar.Vertices.Count];
        TensorPoint tp;


        for (int i = 0; i < MScalar.Vertices.Count; i++)
        {
            tp = new TensorPoint(MScalar.Vertices[i], VectorFromColor(MVector.VertexColors[i]), MScalar.VertexColors[i].GetBrightness());
            MeshField[i] = tp;
        }

        return MeshField;
    }

    /// <summary>
    /// This function converts a 0-255 RGB color into a -1 to 1 Vector3d
    /// </summary>
    /// <param name="col"></param>
    /// <returns>Vector3d in a -1 to 1 range</returns>
    public Vector3d VectorFromColor(Color col)
    {
        Vector3d vCol = new Vector3d();

        vCol = new Vector3d(
            Map(col.R, 0.0, 255.0, -1.0, 1.0),
            Map(col.G, 0.0, 255.0, -1.0, 1.0),
            Map(col.B, 0.0, 255.0, -1.0, 1.0)
            );
        return vCol;
    }

    /// <summary>
    /// This Function maps a value from a source domain to a destination domain
    /// </summary>
    /// <param name="val">the value</param>
    /// <param name="fromMin">low value of source domain</param>
    /// <param name="fromMax">high value of source domain</param>
    /// <param name="toMin">low value of destination domain</param>
    /// <param name="toMax">high value of destination domain</param>
    /// <returns>the remapped value</returns>
    public double Map(double val, double fromMin, double fromMax, double toMin, double toMax)
    {
        return toMin + (val - fromMin) * (toMax - toMin) / (fromMax - fromMin);
    }


    // </Custom additional code> 

    private List<string> __err = new List<string>(); //Do not modify this list directly.
    private List<string> __out = new List<string>(); //Do not modify this list directly.
    private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
    private IGH_ActiveObject owner;                  //Legacy field.
    private int runCount;                            //Legacy field.

    public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
    {
        //Prepare for a new run...
        //1. Reset lists
        this.__out.Clear();
        this.__err.Clear();

        this.Component = owner;
        this.Iteration = iteration;
        this.GrasshopperDocument = owner.OnPingDocument();
        this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

        this.owner = this.Component;
        this.runCount = this.Iteration;
        this.doc = this.RhinoDocument;

        //2. Assign input parameters
        object x = default(object);
        if (inputs[0] != null)
        {
            x = (object)(inputs[0]);
        }

        object y = default(object);
        if (inputs[1] != null)
        {
            y = (object)(inputs[1]);
        }



        //3. Declare output parameters
        object A = null;


        //4. Invoke RunScript
        RunScript(x, y, ref A);

        try
        {
            //5. Assign output parameters to component...
            if (A != null)
            {
                if (GH_Format.TreatAsCollection(A))
                {
                    IEnumerable __enum_A = (IEnumerable)(A);
                    DA.SetDataList(1, __enum_A);
                }
                else
                {
                    if (A is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(A));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, A);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }

        }
        catch (Exception ex)
        {
            this.__err.Add(string.Format("Script exception: {0}", ex.Message));
        }
        finally
        {
            //Add errors and messages... 
            if (owner.Params.Output.Count > 0)
            {
                if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
                {
                    List<string> __errors_plus_messages = new List<string>();
                    if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
                    if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
                    if (__errors_plus_messages.Count > 0)
                        DA.SetDataList(0, __errors_plus_messages);
                }
            }
        }
    }
}