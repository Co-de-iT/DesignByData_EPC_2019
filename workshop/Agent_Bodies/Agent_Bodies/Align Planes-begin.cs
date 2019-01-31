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
using System.Collections.Concurrent;
using System.Drawing;
using SimplexNoise;
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
    private void RunScript(bool reset, bool on, List<Point3d> P, List<Vector3d> V, double cNS, double cNT, double pR, double cI, double aI, double sI, double fI, double mF, ref object Planes, ref object MeshPlanes)
    {
        // <Custom code> 

        // align planes with field - C#
        // code by Alessio Erioli - (c) Co-de-iT 2019 
        //  implementations:
        //  . fixed pts and dirs (for future modularization of the process over larger number of units
        //  . field as static set of pts and vectors (RTree searchable)


        if (P == null || P.Count == 0) return;

        if (reset || aPS == null)
        {
            // passing the essential parameters to the new simulation
            aPS = new AgentPlaneSimulation(P, V);
            //aPS = new AgentPlaneSimulation(P, V, cNS, cNT, pR, cI, aI, sI, fI, mF);

            // initializing arrays for export
            gP = new GH_Plane[aPS.agentPlanes.Length];
            gM = new GH_Mesh[aPS.agentPlanes.Length];
        }
        else
        {
            // update parameters
            aPS.cNS = cNS;
            aPS.cNT = cNT;
            aPS.pR = pR;
            aPS.cI = cI;
            aPS.aI = aI;
            aPS.sI = sI;
            aPS.fI = fI;
            aPS.mF = mF;

            // run simulation
            aPS.Run();

            // extract output geometries and information

            Parallel.For(0, aPS.agentPlanes.Length, i =>
            {
                // extract planes
                gP[i] = new GH_Plane(aPS.agentPlanes[i].PlaneOut());
                // extract meshes (optional)
                //gM[i] = new GH_Mesh(aPS.agentPlanes[i].MeshOut(pR));
            });


            Planes = gP;
            // MeshPlanes = gM;

            if (on) Component.ExpireSolution(true); // equivalent to a timer
        }
        // </Custom code> 
    }

    // <Custom additional code> 

    // ............................................................ global variables 
    // declaring global variables for geometry output
    public GH_Plane[] gP;
    public GH_Mesh[] gM;
    public AgentPlaneSimulation aPS;

    // .................................................................... classes 
    // .............................................................................
    // .............................................................................



    // ...................................................... Simulation ..........
    // ............................................................................
    // ............................................................................

    public class AgentPlaneSimulation
    {
        // ..........................    fields

        public double cNS;
        public double cNT;
        public double pR;
        public double cI;
        public double aI;
        public double sI;
        public double fI;
        public double mF;

        public AgentPlane[] agentPlanes;

        // ..........................    constructor

        public AgentPlaneSimulation(List<Point3d> P, List<Vector3d> V, double cNS, double cNT, double pR, double cI, double aI, double sI, double fI, double mF)
        {

            this.cNS = cNS;
            this.cNT = cNT;
            this.pR = pR;
            this.cI = cI;
            this.aI = aI;
            this.sI = sI;
            this.fI = fI;
            this.mF = mF;

            // build agent planes array
            agentPlanes = new AgentPlane[P.Count];

            Parallel.For(0, P.Count, i =>
            {
                agentPlanes[i] = new AgentPlane(P[i], V[i], this);
            });

        }

        public AgentPlaneSimulation(List<Point3d> P, List<Vector3d> V)
        {
            // build agent planes array
            agentPlanes = new AgentPlane[P.Count];

            Parallel.For(0, P.Count, i =>
            {
                agentPlanes[i] = new AgentPlane(P[i], V[i], this);
            });

        }

        // ..........................    methods

        public void Run()
        {
            UpdateAgents();
        }

        public void UpdateAgents()
        {

            // . . . . . . . . . . environmental and stigmergic interactions
            

            // calculate field influence vector for agents

            Vector3d[] curlNoiseVector = new Vector3d[agentPlanes.Length];

            var p = Partitioner.Create(0, agentPlanes.Length);

            Parallel.ForEach(p, (range, loopState) =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    // calculate curl noise vector
                    CurlNoise(agentPlanes[i].O, cNS, cNT, true, out curlNoiseVector[i]);
                    curlNoiseVector[i].Unitize();

                    // this resets the acceleration (or desired direction)
                    // in case a different update sequence is implemented
                    // remember to reset desired before calculating the new iteration
                    agentPlanes[i].ResetDesired();
                    agentPlanes[i].ComputeFieldAlign(curlNoiseVector[i], fI);
                }

            });

            // . . . . . . . . . . peer-to-peer interaction
            FLockRTree();

            // . . . . . . . . . . update position and direction
            UpdateAgentsDirection();

        }

        public void UpdateAgents_OLD()
        {
            // calculate curl noise vector for agents

            Vector3d[] curlNoiseVector = new Vector3d[agentPlanes.Length];

            var p = Partitioner.Create(0, agentPlanes.Length);

            Parallel.ForEach(p, (range, loopState) =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    CurlNoise(agentPlanes[i].O, cNS, cNT, true, out curlNoiseVector[i]);
                    curlNoiseVector[i].Unitize();
                    agentPlanes[i].ResetDesired();
                    agentPlanes[i].AlignWithField(curlNoiseVector[i], fI);
                }

            });

            //Parallel.For(0, agentPlanes.Length, i =>
            //{
            //    CurlNoise(agentPlanes[i].O, cNS, cNT, true, out curlNoiseVector[i]);
            //    curlNoiseVector[i].Unitize();
            //    agentPlanes[i].Align(curlNoiseVector[i], mF);
            //});


        }

        public void FLockRTree()
        {
            // define neighborhood radius
            double nR = pR * 3;

            // declare RTree
            RTree rTree = new RTree();

            // populate RTree
            for (int i = 0; i < agentPlanes.Length; i++)
                rTree.Insert(agentPlanes[i].O, i);

            foreach (AgentPlane agent in agentPlanes)
            {
                List<AgentPlane> neighbours = new List<AgentPlane>();

                EventHandler<RTreeEventArgs> rTreeCallback =
                    (object sender, RTreeEventArgs args) =>
                    {
                        if (agentPlanes[args.Id] != agent)
                            neighbours.Add(agentPlanes[args.Id]);
                    };

                rTree.Search(new Sphere(agent.O, nR), rTreeCallback);

                // compute desired vector for each agent
                agent.ComputeDesiredNeighbours(neighbours);
            }

        }

        public void UpdateAgentsDirection()
        {
            // here's one of my mods - update positions and velocities in parallel
            var part = Partitioner.Create(0, agentPlanes.Length);
            Parallel.ForEach(part, (range, loopstate) =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    // update at max Force
                    agentPlanes[i].Update(mF);
                }
            });
            //Parallel.ForEach(Agents, agent => { agent.UpdateVelocityAndPosition(); });
            //foreach (FlockAgent agent in Agents) agent.UpdateVelocityAndPosition();
        }

    }


    // ........................................................... Agent ..........
    // ............................................................................
    // ............................................................................
    public class AgentPlane
    {

        // fields
        public Point3d O;
        public Vector3d X, Y, Z;
        public Vector3d desDir; // desired direction - influenced by environmental field and nearby agents' directions
        public Vector3d desPos; // desired position - influenced by environmental movement constraints (es. surface) and neighbour agents
        public AgentPlaneSimulation agentSim;

        // constructor
        public AgentPlane(Point3d O, Vector3d dirX, AgentPlaneSimulation agentSim)
        {
            this.O = O;
            dirX.Unitize();
            this.X = dirX;
            this.agentSim = agentSim;
        }
        // methods

        public void Update(double mF)
        {
            // update position
            O += desPos * mF;
            // update direction
            X = X * (1- mF) + mF * desDir;
            X.Unitize();

        }

        public void ResetDesired()
        {
            desDir = Vector3d.Zero;
            desPos = Vector3d.Zero;
        }

        public void ComputeDesiredNeighbours(List<AgentPlane> neighbours)
        {

            // neighbours interaction
            if (neighbours.Count != 0)
            {

                // ............................ alignment behavior

                // align direction with neighbours
                Vector3d align = Vector3d.Zero;

                foreach (AgentPlane neighbour in neighbours)
                    align += neighbour.X;

                align /= neighbours.Count;

                // updates desired direction
                // multiplies alignment vector by alignment intensity factor
                desDir += agentSim.aI * align;

                // ............................ cohesion behavior
                Vector3d cohesion = Vector3d.Zero;
                foreach (AgentPlane neighbour in neighbours)
                    cohesion += (Vector3d)Point3d.Subtract(neighbour.O, O);

                cohesion /= neighbours.Count;

                // updates desired position
                // multiplies cohesion vector by cohesion intensity factor
                desPos += agentSim.cI * cohesion;


                // ............................ separation behavior
                Vector3d separation = Vector3d.Zero;
                int sepCount = 0;
                double sepDistSq = agentSim.pR * agentSim.pR;
                double distSq;
                foreach (AgentPlane neighbour in neighbours)
                {
                    distSq = O.DistanceToSquared(neighbour.O);
                    if (distSq < sepDistSq)
                    {
                        // separation vector is bigger when closer to another agent
                        separation += (Vector3d)Point3d.Subtract(O, neighbour.O) * (sepDistSq-distSq);
                        sepCount++;
                    }
                }
                if (sepCount > 0)
                    separation /= sepCount;

                // updates desired position
                // multiplies separation vector by separation intensity factor
                desPos += agentSim.sI * separation;

            }

        }

        public void AlignWithField(Vector3d fieldVector, double fI)
        {

            X = X * (1 - fI) + fieldVector * fI;
            X.Unitize();
        }

        public void ComputeFieldAlign(Vector3d fieldVector, double fI)
        {
            desDir += fieldVector * fI;
        }

        public Plane PlaneOut()
        {
            Vector3d oV = new Vector3d(O);
            oV.Unitize();
            this.Y = Vector3d.CrossProduct(X, oV);
            return new Plane(O, X, Y);
        }

        public GH_Mesh MeshOut(double rad)
        {
            Plane pl = PlaneOut();
            Transform x = Transform.PlaneToPlane(Plane.WorldXY, pl);

            Mesh m = new Mesh();
            Point3d[] verts = new Point3d[] { new Point3d(-1, -1, 0), new Point3d(1, -1, 0), new Point3d(1, 1, 0), new Point3d(-1, 1, 0) };
            m.Vertices.AddVertices(verts);
            m.Faces.AddFace(0, 1, 2, 3);
            m.Scale(rad);
            m.Transform(x);
            m.Normals.ComputeNormals();

            GH_Mesh m1 = new GH_Mesh(m);
            return m1;
        }
    }

    // .................................................................. Utilities 
    // .............................................................................
    // 
    // .............................................................................
    // ..................... 3D Curl Noise function ................................
    // .............................................................................

    static void CurlNoise(Point3d P, double S, double t, bool flag, out Vector3d V)
    {
        // . . . . . . . . Points . . scale . .  time . .  3D . . . .  out vectors

        float fS = (float)S;
        float fT = (float)t;
        float nX = (float)P.X * fS + fT; // num5
        float nY = (float)P.Y * fS + fT; // num6
        float nZ = (float)P.Z * fS + fT; // num7

        double num8 = Noise.Generate(nX, nY, nZ);
        float dT = 1f;
        float nPlus = Noise.Generate(nX, nY + dT, nZ);
        float nMinus = Noise.Generate(nX, nY - dT, nZ);
        float nDiff = (nPlus - nMinus) / (2f * dT);
        nPlus = Noise.Generate(nX, nY, nZ + dT);
        nMinus = Noise.Generate(nX, nY, nZ - dT);
        float num13 = (nPlus - nMinus) / (2f * dT);
        float num14 = nDiff - num13;
        nX -= fT;
        nY -= fT;
        nPlus = Noise.Generate(nX, nY, nZ + dT);
        nMinus = Noise.Generate(nX, nY, nZ - dT);
        nDiff = (nPlus - nMinus) / (2f * dT);
        nPlus = Noise.Generate(nX + dT, nY, nZ);
        nMinus = Noise.Generate(nX - dT, nY, nZ);
        num13 = (nPlus - nMinus) / (2f * dT);
        float num15 = nDiff - num13;
        nPlus = Noise.Generate(nX + dT, nY, nZ);
        nMinus = Noise.Generate(nX - dT, nY, nZ);
        nDiff = (nPlus - nMinus) / (2f * dT);
        nPlus = Noise.Generate(nX, nY + dT, nZ);
        nMinus = Noise.Generate(nX, nY - dT, nZ);
        num13 = (nPlus - nMinus) / (2f * dT);
        float num16 = nDiff - num13;
        Vector3d val2 = new Vector3d((double)num14, (double)num15, (double)num16);
        if (!flag)
        {
            nPlus = Noise.Generate(nX, nY + dT);
            nMinus = Noise.Generate(nX, nY - dT);
            nDiff = (nPlus - nMinus) / (2f * dT);
            nPlus = Noise.Generate(nX + dT, nY);
            nMinus = Noise.Generate(nX - dT, nY);
            nDiff = (nPlus - nMinus) / (2f * dT);
            val2 = new Vector3d((double)nDiff, (double)(0f - num13), 0.0);
        }
        V = val2;

    }

    public float Remap(float val, float from1, float to1, float from2, float to2)
    {
        return (val - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    public GH_Colour Vec2Col(Vector3d v)
    {
        v.Unitize();
        int red = (int)Math.Floor(Remap((float)v.X, -1f, 1f, 0f, 255f));
        int green = (int)Math.Floor(Remap((float)v.Y, -1f, 1f, 0f, 255f));
        int blue = (int)Math.Floor(Remap((float)v.Z, -1f, 1f, 0f, 255f));
        return new GH_Colour(Color.FromArgb(red, green, blue));
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
        List<Point3d> P = null;
        if (inputs[0] != null)
        {
            P = GH_DirtyCaster.CastToList<Point3d>(inputs[0]);
        }
        List<Vector3d> V = null;
        if (inputs[1] != null)
        {
            V = GH_DirtyCaster.CastToList<Vector3d>(inputs[1]);
        }
        List<Vector3d> vF = null;
        if (inputs[2] != null)
        {
            vF = GH_DirtyCaster.CastToList<Vector3d>(inputs[2]);
        }
        double pR = default(double);
        if (inputs[3] != null)
        {
            pR = (double)(inputs[3]);
        }

        double cI = default(double);
        if (inputs[4] != null)
        {
            cI = (double)(inputs[4]);
        }

        double aI = default(double);
        if (inputs[5] != null)
        {
            aI = (double)(inputs[5]);
        }

        double sI = default(double);
        if (inputs[6] != null)
        {
            sI = (double)(inputs[6]);
        }

        object fI = default(object);
        if (inputs[7] != null)
        {
            fI = (object)(inputs[7]);
        }

        double mF = default(double);
        if (inputs[8] != null)
        {
            mF = (double)(inputs[8]);
        }



        //3. Declare output parameters
        object Planes = null;


        //4. Invoke RunScript
        RunScript(P, V, vF, pR, cI, aI, sI, fI, mF, ref Planes);

        try
        {
            //5. Assign output parameters to component...
            if (Planes != null)
            {
                if (GH_Format.TreatAsCollection(Planes))
                {
                    IEnumerable __enum_Planes = (IEnumerable)(Planes);
                    DA.SetDataList(1, __enum_Planes);
                }
                else
                {
                    if (Planes is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(Planes));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, Planes);
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