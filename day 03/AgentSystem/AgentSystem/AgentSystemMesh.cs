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
    private void RunScript(bool reset, bool go, Mesh M, List<Vector3d> MVf, double meI, List<Point3d> P, List<Vector3d> V, double nR, double coS, double alS, double seS, double seR, double tSpac, ref object Ap, ref object Av, ref object neigh, ref object Tr)
    {
        // <Custom code>

        GH_Point[] ptsOut;
        GH_Vector[] vecOut;
        Polyline[] trails;
        DataTree<GH_Point> neighOut;

        if (reset || AgSys == null /*|| MPoints == null*/)
        {
            AgSys = new AgentSystem(P, V);
            //MPoints = PointCloudFromMesh(M);
            Menv = M;
            MeshRTree = RTreeFromMesh(M);
            MeshField = BuildMeshField(M, MVf);
            //ptsOut = new GH_Point[Flock.Agents.Count];
            //vecOut = new GH_Vector[Flock.Agents.Count];
        }

        if (go)
        {
            // update parameters
            AgSys.NeighborhoodRadius = nR;
            AgSys.CohesionStrength = coS;
            AgSys.AlignmentStrength = alS;
            AgSys.SeparationStrength = seS;
            AgSys.SeparationRadius = seR;
            AgSys.MeshStrength = meI;
            AgSys.TrailSpacing = tSpac;
            AgSys.futPosMultiplier = 1.5;
            AgSys.MeshSeekRadius = 5.0;

            // update system
            //AgSys.Update();
            AgSys.UpdateRTree();
            Component.ExpireSolution(true);
        }

        AgSys.GetPtsVecs(out ptsOut, out vecOut);
        neighOut = GetNeighbours(AgSys);
        trails = AgSys.GetTrails();

        Ap = ptsOut;
        Av = vecOut;
        neigh = neighOut;
        Tr = trails;


        // </Custom code>
    }

    // <Custom additional code> 

    // global variables
    public AgentSystem AgSys;
    public static TensorPoint[] MeshField;
    public static Mesh Menv;
    public static RTree MeshRTree;

    // classes

    public class AgentSystem
    {
        public List<Agent> Agents;
        public List<Polyline> AgentsTrails;
        public double NeighborhoodRadius;
        public double CohesionStrength;
        public double AlignmentStrength;
        public double SeparationStrength;
        public double SeparationRadius;
        public double MaxSpeed;
        public double BoundingBoxSize;
        public double ContainmentStrength;
        public double MeshStrength;
        public double TrailSpacing;
        public double futPosMultiplier;
        public double MeshSeekRadius;
        public double SeekColorStrength;

        public AgentSystem(List<Point3d> P, List<Vector3d> V)
        {
            Agents = new List<Agent>();

            for (int i = 0; i < P.Count; i++)
            {
                Agent ag = new Agent(P[i], V[i]);
                ag.Agents = this;

                Agents.Add(ag);
            }

            MaxSpeed = 0.3;
            BoundingBoxSize = 30.0;
            ContainmentStrength = 1.0;
            SeekColorStrength = 5.0;
        }

        public void Update()
        {
            // . . . . . . . . . . . . . . . . . . . . . . step 1 - calculate desired
            /*
             function(param a, param b, ....) {...}

            (param a, param b, ....) => {...}
             
             */
            Parallel.ForEach(Agents, ag =>
            {
                // find neighbours & compute desired velocity for each agent

                ComputeAgentDesiredVelocity(ag);
            });

            //foreach (Agent ag in Agents)
            //{
            //    // find neighbours & compute desired velocity for each agent
            //    ComputeAgentDesiredVelocity(ag);
            //}

            // . . . . . . . . . . . . . . . . . . . . . . step 2 - update agents velocity and position
            foreach (Agent ag in Agents) ag.UpdateVelocityAndPosition();
        }


        public void UpdateRTree()
        {

            foreach (Agent ag in Agents)
            {
                // clear neighbours list
                ag.neighbours.Clear();
                ag.neighTens.Clear();

                // Eventhandler function for RTree search
                EventHandler<RTreeEventArgs> rTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    ag.neighbours.Add(Menv.Vertices[args.Id]);
                    ag.neighTens.Add(MeshField[args.Id]);
                };

                MeshRTree.Search(new Sphere(Menv.ClosestPoint(ag.position + ag.velocity * futPosMultiplier), MeshSeekRadius), rTreeCallback);

                ag.ResetDesired();
                ag.SeekMeshNeighbours(MeshStrength);
                ag.SeekColor(SeekColorStrength);

            }

            //FlockTrails();


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

        public void FlockTrails()
        {
            List<Polyline> AllTrails = new List<Polyline>();
            foreach (Agent ag in Agents)
                AllTrails.Add(ag.trail);

            // find closest trail point and dir for agent

            foreach (Agent ag in Agents)
            {
                double trailPar;
                Point3d testPoint = new Point3d();
                Point3d nearest = new Point3d();
                Point3d futPos = ag.position + ag.velocity;
                Vector3d nearestDir = new Vector3d();
                double dd;
                double MinD = Double.MaxValue;
                foreach (Polyline tr in AllTrails)
                {
                    // if it's not the trail of the current agent....
                    if (AllTrails.IndexOf(tr) != Agents.IndexOf(ag))
                    {
                        // evaluate closest point and its distance
                        trailPar = tr.ClosestParameter(futPos);
                        testPoint = tr.PointAt(trailPar);
                        dd = testPoint.DistanceToSquared(futPos);
                        // if closer that the min distance
                        if (dd < MinD)
                        {
                            MinD = dd;
                            nearest = new Point3d(testPoint);
                            nearestDir = tr.TangentAt(trailPar);
                        }
                    }

                }

                ag.FlockSinglePoint(nearest, nearestDir);
            }
        }

        public List<Agent> FindNeighbours(Agent ag)
        {
            List<Agent> neighbours = new List<Agent>();

            foreach (Agent neighbour in Agents)
            {
                if (neighbour != ag && neighbour.position.DistanceTo(ag.position) < NeighborhoodRadius)
                    neighbours.Add(neighbour);
            }

            return neighbours;
        }

        public void ComputeAgentDesiredVelocity(Agent ag)
        {
            List<Agent> neighbours = FindNeighbours(ag);
            ag.ResetDesired();
            ag.ComputeDesiredVelocity(neighbours);
        }

        // extract points and vectors for output
        public void GetPtsVecs(out GH_Point[] pts, out GH_Vector[] vecs)
        {
            pts = new GH_Point[Agents.Count];
            vecs = new GH_Vector[Agents.Count];

            for (int i = 0; i < Agents.Count; i++)
            {
                pts[i] = new GH_Point(Agents[i].position);
                vecs[i] = new GH_Vector(Agents[i].velocity);
            }
        }

        public Polyline[] GetTrails()
        {
            Polyline[] trails = new Polyline[Agents.Count];

            Parallel.For(0, Agents.Count, i =>
              {
                  trails[i] = Agents[i].trail;
              });
            return trails;
        }

    }

    public class Agent
    {
        // field
        public Point3d position;
        public Vector3d velocity;
        public Vector3d desiredVelocity;
        public Polyline trail;
        public AgentSystem Agents;
        public List<Point3d> neighbours;
        public List<TensorPoint> neighTens;

        // constructor
        public Agent(Point3d position, Vector3d velocity)
        {
            this.position = position;
            this.velocity = velocity;
            desiredVelocity = this.velocity;
            trail = new Polyline { this.position };
            neighbours = new List<Point3d>();
            neighTens = new List<TensorPoint>();
        }

        // methods
        public void ResetDesired()
        {
            desiredVelocity = velocity;
        }

        public void ComputeDesiredVelocity(List<Agent> neighbours)
        {
            // ------------------------------- CONTAINMENT -------------------------------
            //Containment();
            // ------------------------------- FLOCKING -------------------------------
            if (neighbours.Count > 0)
            {
                Flock(neighbours);
            }

            // ------------------------------- POINT CLOUD or MESH BEHAVIOUR -------------------------------
            //SeekPointCloud(MPoints);
            //SeekMeshClosestPt(Menv);
            // ------------------------------- FIELD BEHAVIOUR -------------------------------

            // ------------------------------- CUSTOM MOVEMENT BEHAVIOUR ---------------------
        }

        public void Containment()
        {


            if (position.X < 0.0)
                desiredVelocity += new Vector3d(-position.X, 0.0, 0.0);
            else if (position.X > Agents.BoundingBoxSize)
                desiredVelocity += new Vector3d(Agents.BoundingBoxSize - position.X, 0, 0);

            if (position.Y < 0.0)
                desiredVelocity += new Vector3d(0.0, -position.Y, 0.0);
            else if (position.Y > Agents.BoundingBoxSize)
                desiredVelocity += new Vector3d(0.0, Agents.BoundingBoxSize - position.Y, 0.0);

            if (position.Z < 0.0)
                desiredVelocity += new Vector3d(0.0, 0.0, -position.Z);
            else if (position.Z > Agents.BoundingBoxSize)
                desiredVelocity += new Vector3d(0.0, 0.0, Agents.BoundingBoxSize - position.Z);

            desiredVelocity *= Agents.ContainmentStrength;
        }

        public void Flock(List<Agent> neighbours)
        {
            // ................................... COHESION BEHAVIOUR .....................
            //
            // find neighbours average
            Point3d average = new Point3d();

            foreach (Agent neighbour in neighbours)
                average += neighbour.position;

            average /= neighbours.Count;

            // go there
            Vector3d cohesion = average - position;

            desiredVelocity += Agents.CohesionStrength * cohesion;

            // ................................... ALIGNMENT BEHAVIOUR .....................
            //
            Vector3d alignment = Vector3d.Zero;

            foreach (Agent neighbour in neighbours)
                alignment += neighbour.velocity;

            alignment /= neighbours.Count;

            desiredVelocity += alignment * Agents.AlignmentStrength;

            // ................................... SEPARATION BEHAVIOUR .....................
            //
            Vector3d separation = Vector3d.Zero;

            foreach (Agent neighbour in neighbours)
            {
                double distanceToNeighbour = position.DistanceTo(neighbour.position);
                if (distanceToNeighbour < Agents.SeparationRadius)
                {
                    Vector3d getAway = position - neighbour.position;
                    separation += getAway /= (getAway.Length * distanceToNeighbour);
                }

            }

            desiredVelocity += separation * Agents.SeparationStrength;

        }

        public void FlockSinglePoint(Point3d target, Vector3d targetVel)
        {
            // ................................... COHESION BEHAVIOUR .....................
            //
            // go there
            Vector3d cohesion = target - position;

            desiredVelocity += Agents.CohesionStrength * cohesion;

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

        public void SeekPoint(Point3d target, double intensity)
        {
            Vector3d seek = target - position;
            seek.Unitize();
            seek *= Agents.MaxSpeed;

            desiredVelocity += seek * intensity;
        }

        public void SeekPointCloud(PointCloud pc)
        {
            Point3d futurePos = (Point3d)(position + velocity * 3.5);
            int pcIndex = pc.ClosestPoint(futurePos);
            Point3d p = pc[pcIndex].Location;

            SeekPoint(p, Agents.MeshStrength);

        }

        public void SeekMeshClosestPt(Mesh M)
        {
            Point3d mP = M.ClosestPoint(position + velocity * 1.5);
            SeekPoint(mP, Agents.MeshStrength);

        }

        public void SeekMeshNeighbours(double seekIntensity)
        {
            //desiredVelocity = velocity;
            if (neighbours.Count > 0)
            {

                Point3d average = new Point3d();

                foreach (Point3d n in neighbours)
                {
                    average += n;
                }

                average /= neighbours.Count;
                SeekPoint(average, seekIntensity);
            }
        }

        public void SeekColor(double seekIntensity)
        {

            if (neighTens.Count > 0)
            {
                double bri;
                double MaxBri = -1.0;
                int MaxInd = -1;

                // find point with max intensity
                for (int i = 0; i < neighTens.Count; i++)
                {
                    bri = neighTens[i].scalar; // brightness is 0-1 and RGB 0-255
                    if (bri > MaxBri)
                    {
                        MaxBri = bri;
                        MaxInd = i;
                    }
                }

                Vector3d seek = (neighTens[MaxInd].position - position) * 0.08 + neighTens[MaxInd].vector * 0.92;
                seek.Unitize();
                seek *= Agents.MaxSpeed;

                desiredVelocity += seek * seekIntensity;

            }
        }

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

            position +=velocity;

            if (position.DistanceTo(trail[trail.Count - 1]) > Agents.TrailSpacing)
                trail.Add(position);
            //trail.Add(position);

        }

        public List<GH_Point> GetNeighbours()
        {
            List<GH_Point> neighOut = new List<GH_Point>();

            foreach (Point3d n in neighbours)
                neighOut.Add(new GH_Point(n));

            return neighOut;
        }

    }

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

    // utilities functions
    public PointCloud PointCloudFromMesh(Mesh M)
    {
        PointCloud pc = new PointCloud();

        for (int i = 0; i < M.Vertices.Count; i++)
        {
            pc.Add(M.Vertices[i], M.Normals[i]);
        }

        return pc;
    }

    public RTree RTreeFromMesh(Mesh M)
    {

        return RTree.CreateFromPointArray(M.Vertices.ToPoint3dArray());
        //RTree MPoints = new RTree();

        //for (int i = 0; i < M.Vertices.Count; i++)
        //    MPoints.Insert(M.Vertices[i], i);

        //return MPoints;
    }

    public TensorPoint[] BuildMeshField(Mesh M, List<Vector3d> VectorField)
    {
        TensorPoint[] MeshField = new TensorPoint[M.Vertices.Count];
        TensorPoint tp;
        for (int i = 0; i < M.Vertices.Count; i++)
        {
            tp = new TensorPoint(M.Vertices[i], VectorField[i], M.VertexColors[i].GetBrightness());
            MeshField[i] = tp;
        }

        return MeshField;
    }

    public DataTree<GH_Point> GetNeighbours(AgentSystem AgSys)
    {
        DataTree<GH_Point> neighOut = new DataTree<GH_Point>();

        for (int i = 0; i < AgSys.Agents.Count; i++)
        {
            neighOut.AddRange(AgSys.Agents[i].GetNeighbours(), new GH_Path(i));
        }

        return neighOut;
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