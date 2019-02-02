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
using System.Collections.Concurrent;
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
    private void RunScript(bool reset, bool go, Mesh M, List<Point3d> P, List<Vector3d> Vf, double sR, double sI, ref object points, ref object vectors, ref object tr, ref object cols, ref object neigh, ref object neighCol, ref object outMCol)
    {
        // <Custom code> 
        // ............................................................................

        // return on null input
        if (M == null || P == null) return;

        // initialize on reset
        if (reset || PS == null)
        {
            PS = new ParticleSystem(P, M);
            scalarField = PopulateScalarField(M);
            vectorField = Vf.ToArray();
            MeshPoints = PopulatePointCloud(M);
            debug = "";
        }

        if (go)
        {
            PS.seekRadius = sR;
            PS.seekIntensity = sI;
            PS.futPosMult = 3.0f;
            PS.UpdateRTree(M);
            Component.ExpireSolution(true);
            //EvaporateMesh(M, 0.65);
            EvaporateField(scalarField, 0.98);
        }

        // extract geometries

        PS.GetPointsVectors(out pts, out vecs);

        // Print(debug);
        
        debug = "";
        points = pts;
        vectors = vecs;
        neigh = PS.GetNeighPts();
        neighCol = PS.GetNeighBrightness();
        //if (!go)
            outMCol = GetScalarField(scalarField);
        //tr = PS.GetTrails();
        /* see this line above and the one after this commentary?
         
         This made me bleed:
         
         trails are Polylines, but I initialize them with a single point (so that's not a Polyline... yet)
         This causes a 'reference not set to an instance' exception (when something is declared but not instanced)
         the problem is that everything below this line does not compile anymore BUT THE SCRIPT EXECUTES
         so I coudn't understand why neighPts contained data but nothig was outputting.... GROAN!!!

        */
        //neigh = neighPts;


        // ............................................................................
        // </Custom code> 
    }

    // <Custom additional code> 
    // ............................................................................Global Variables
    public ParticleSystem PS;
    public GH_Point[] pts;
    public GH_Vector[] vecs;
    public Polyline[] trails;
    public static Vector3d[] vectorField;
    public static float[] scalarField;
    public static PointCloud MeshPoints;

    public static string debug;

    // ............................................................................Classes

    // .......................................................... Particle System ........

    public class ParticleSystem
    {
        // . . . . . . . . . . . . . . . . . . . . . . fields
        public List<Particle> Particles;
        public RTree MeshRTree;
        public double seekRadius;
        public double seekIntensity;
        public double futPosMult;

        // . . . . . . . . . . . . . . . . . . . . . . constructor
        public ParticleSystem(List<Point3d> positions, Mesh M)
        {
            Particles = new List<Particle>();

            for (int i = 0; i < positions.Count; i++)
                Particles.Add(new Particle(positions[i], RandomVector(i) * 1.5));

            MeshRTree = PopulateMeshRTree(M);
        }

        // . . . . . . . . . . . . . . . . . . . . . . methods

        public void Update(Mesh M)
        {

            foreach (Particle p in Particles)
                p.SeekMeshClosestPt(M, seekIntensity);
            foreach (Particle p in Particles)
                p.Update();

        }

        public void UpdateParallel(Mesh M)
        {

            var x = Partitioner.Create(0, Particles.Count);

            Parallel.ForEach(x, (range, loopstate) =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    Particles[i].SeekMeshClosestPt(M, seekIntensity);
                    Particles[i].Update();
                }
            });

        }

        public void UpdateParallelSimple(Mesh M)
        {
            Parallel.ForEach(Particles, p =>
            {
                //foreach (Particle p in Particles)
                p.SeekMeshClosestPt(M, seekIntensity);
                p.Update();
            });
            //foreach (Particle p in Particles)
            //    p.Update();

        }

        public void UpdateRTree(Mesh M)
        {
            //IEnumerable<int[]> index;
            //int[] VertexNumbers;
            //int vN;
            foreach (Particle p in Particles)
            {
                // clear particle neighbours list
                p.neighbours.Clear();
                p.neighFieldPts.Clear();

                // Eventhandler function for RTree search
                EventHandler<RTreeEventArgs> rTreeCallback = (object sender, RTreeEventArgs args) =>
                {
                    p.neighbours.Add(M.Vertices[args.Id]);
                    p.neighFieldPts.Add(new FieldPt(args.Id, scalarField[args.Id]));
                };

                MeshRTree.Search(new Sphere(M.ClosestPoint(p.pos + p.vel * futPosMult), seekRadius), rTreeCallback);

                // p.SeekNeighbours(seekIntensity);
                p.SeekColor(seekIntensity);

                //// query RTree of Mesh vertices for closest point to p.pos (returns an IEnumerable)
                //index = RTree.Point3dClosestPoints(M.Vertices.ToPoint3dArray(), new List<Point3d>{ p.pos}, 200);
                //// get enumerator
                //IEnumerator en = index.GetEnumerator();
                //// get first array of indices
                //en.MoveNext();
                //VertexNumbers = (int[]) en.Current;
                //vN = VertexNumbers[0];


                //// update scalar field
                scalarField[MeshPoints.ClosestPoint(p.pos)] += p.PheroStrength;

                
            }

            foreach (Particle p in Particles)
            {
                

            }

            if (Particles.Count > 500)
            {
                Parallel.ForEach(Particles, p =>
               {
                   
                   p.Update();
               });
            }
            else
                foreach (Particle p in Particles)
                    p.Update();

        }

        public void GetPointsVectors(out GH_Point[] pts, out GH_Vector[] vecs)
        {
            pts = new GH_Point[Particles.Count];
            vecs = new GH_Vector[Particles.Count];

            for (int i = 0; i < Particles.Count; i++)
            {
                pts[i] = new GH_Point(Particles[i].pos);
                vecs[i] = new GH_Vector(Particles[i].vel);
            }
        }

        public Polyline[] GetTrails()
        {
            Polyline[] trails = new Polyline[Particles.Count];
            for (int i = 0; i < Particles.Count; i++)
            {
                trails[i] = Particles[i].trail;
            }
            return trails;
        }

        public DataTree<GH_Point> GetNeighPts()
        {
            DataTree<GH_Point> ptsOut = new DataTree<GH_Point>();

            for (int i = 0; i < Particles.Count; i++)
                ptsOut.EnsurePath(new GH_Path(i));

            // parallelize for more than 500 particles
            if (Particles.Count > 500)
            {

                var x = Partitioner.Create(0, Particles.Count);

                Parallel.ForEach(x, (range, loopstate) =>
                {
                    for (int i = range.Item1; i < range.Item2; i++)
                    {
                        for (int j = 0; j < Particles[i].neighbours.Count; j++)
                            ptsOut.Add(new GH_Point(Particles[i].neighbours[j]), new GH_Path(i));
                    }
                });
            }
            else
            {
                for (int i = 0; i < Particles.Count; i++)
                {
                    for (int j = 0; j < Particles[i].neighbours.Count; j++)
                        ptsOut.Add(new GH_Point(Particles[i].neighbours[j]), new GH_Path(i));
                }
            }

            return ptsOut;
        }

        public DataTree<GH_Vector> GetNeighVectors()
        {

            DataTree<GH_Vector> vecOut = new DataTree<GH_Vector>();

            for (int i = 0; i < Particles.Count; i++)
                vecOut.EnsurePath(new GH_Path(i));

            for (int i = 0; i < Particles.Count; i++)
            {
                for (int j = 0; j < Particles[i].neighFieldPts.Count; j++)
                    vecOut.Add(new GH_Vector(Particles[i].neighFieldPts[j].v), new GH_Path(i));
            }

            return vecOut;
        }

        public DataTree<GH_Number> GetNeighBrightness()
        {

            DataTree<GH_Number> briOut = new DataTree<GH_Number>();

            for (int i = 0; i < Particles.Count; i++)
                briOut.EnsurePath(new GH_Path(i));

            for (int i = 0; i < Particles.Count; i++)
            {
                for (int j = 0; j < Particles[i].neighFieldPts.Count; j++)
                    briOut.Add(new GH_Number(Particles[i].neighFieldPts[j].bri), new GH_Path(i));
            }

            return briOut;
        }
    }

    // .......................................................... Particle ...............
    public class Particle
    {
        public Point3d pos;
        public Vector3d vel;
        public Vector3d acc;
        public double MaxSpeed;
        public float PheroStrength;
        public Polyline trail;
        public List<Point3d> neighbours;
        public List<FieldPt> neighFieldPts;

        public Particle(Point3d pos, Vector3d vel)
        {
            this.pos = pos;
            this.vel = vel;
            acc = Vector3d.Zero;
            MaxSpeed = 1.5f;
            PheroStrength = 0.1f;
            trail = new Polyline { this.pos };
            neighbours = new List<Point3d>();
            neighFieldPts = new List<FieldPt>();
        }

        public void Update()
        {
            Move();
            //trail.Add(pos);
        }

        public void SeekNeighbours(double seekIntensity)
        {
            acc = vel;

            if (neighbours.Count > 0)
            {
                acc = Vector3d.Zero;
                Vector3d desired = Vector3d.Zero;

                foreach (Point3d n in neighbours)
                {
                    desired += n - pos;
                }

                desired /= neighFieldPts.Count;
                desired.Unitize();
                desired *= MaxSpeed;

                acc = (desired - vel) * seekIntensity;
            }
        }

        public void SeekColor(double seekIntensity)
        {
            acc = vel;

            if (neighFieldPts.Count > 0)
            {
                acc = Vector3d.Zero;
                Vector3d desired = Vector3d.Zero;
                double bri;
                double MaxBri = -1.0;
                int MaxInd = -1;

                for (int i = 0; i < neighFieldPts.Count; i++)
                {
                    bri = neighFieldPts[i].bri; // brightness is 0-1 and RGB 0-255
                    if (bri > MaxBri)
                    {
                        MaxBri = bri;
                        MaxInd = i;
                    }


                }
                desired += neighbours[MaxInd] - pos;

                desired /= neighFieldPts.Count;
                desired.Unitize();
                desired *= MaxSpeed;

                acc = (desired - vel) * seekIntensity;
            }
        }

        public void SeekMeshClosestPt(Mesh M, double seekIntensity)
        {
            acc = Vector3d.Zero;
            MeshPoint mP = M.ClosestMeshPoint(pos + vel * 1.5, 100);
            if (mP != null)
            {
                Point3d mCP = mP.Point;

                Vector3d desired = mCP - pos;
                desired.Unitize();
                desired *= MaxSpeed;

                acc = (desired - vel) * seekIntensity;
            }

        }

        public void Move()
        {
            //vel = vel * 0.9 + acc * 0.1;
            vel = vel + acc;
            if (vel.Length > MaxSpeed)
            {
                vel.Unitize();
                vel *= MaxSpeed;
            }
            if (vel.Length < 1)
            {
                vel.Unitize();
            }
            pos += vel;
        }


    }

    public class FieldPt
    {
        public int id;
        public float bri;
        public Vector3d v;

        public FieldPt(int id, float bri, Vector3d v)
        {
            this.id = id;
            this.bri = bri;
            this.v = v;
        }

        public FieldPt(int id, float bri) : this(id, bri, Vector3d.Zero) { }

    }

    // ............................................................................Utilities

    public GH_Number[] GetScalarField(float[] scalarField)
    {
        GH_Number[] outField = new GH_Number[scalarField.Length];

        Parallel.For(0, scalarField.Length, i => 
        {
            outField[i] = new GH_Number(scalarField[i]);
        });

        return outField;
    }

    public static void EvaporateMesh(Mesh M, double evapoRatio)
    {
        if (M.VertexColors.Count == 0) return;


        var x = Partitioner.Create(0, M.VertexColors.Count);

        Parallel.ForEach(x, (range, loopstate) =>
        {
            double b;
            int c;
            for (int i = range.Item1; i < range.Item2; i++)
            {
                b = M.VertexColors[i].R;
                b *= evapoRatio;
                c = (int)b;
                M.VertexColors[i] = Color.FromArgb(c, c, c);
            }
        });

    }

    public static void EvaporateField(float[] scalarField, double evapoRatio)
    {
        float evap = (float)evapoRatio;
        var x = Partitioner.Create(0, scalarField.Length);

        Parallel.For(0, scalarField.Length, i =>
        {
                scalarField[i] *=  evap;
        });
    }

    public float[] PopulateScalarField(Mesh M)
    {
        float[] scalarField = new float[M.Vertices.Count];

        if (M.VertexColors.Count != 0)
        {

            var x = Partitioner.Create(0, M.VertexColors.Count);

            Parallel.ForEach(x, (range, loopstate) =>
            {
                for (int i = range.Item1; i < range.Item2; i++)
                {
                    scalarField[i] = M.VertexColors[i].GetBrightness();

                }
            });
        }
        return scalarField;
    }

    public static RTree PopulateMeshRTree(Mesh M)
    {
        RTree rt = new RTree();

        for (int i = 0; i < M.Vertices.Count; i++)
        {
            rt.Insert((Point3d)M.Vertices[i], i);
        }

        return rt;
    }

    public PointCloud PopulatePointCloud(Mesh M)
    {
        PointCloud mP = new PointCloud();

        for(int i=0; i< M.Vertices.Count;  i++)
        {
            mP.Add(M.Vertices[i], M.Normals[i]);
        }

        return mP;
    }

    public static Vector3d RandomVector(int seed)
    {
        Vector3d r = Vector3d.Zero;
        Random rnd = new Random(seed);
        r = new Vector3d((rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1, (rnd.NextDouble() * 2) - 1);
        return r;
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
        Mesh M = default(Mesh);
        if (inputs[0] != null)
        {
            M = (Mesh)(inputs[0]);
        }

        List<Point3d> P = null;
        if (inputs[1] != null)
        {
            P = GH_DirtyCaster.CastToList<Point3d>(inputs[1]);
        }


        //3. Declare output parameters
        object A = null;


        //4. Invoke RunScript
        RunScript(M, P, ref A);

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