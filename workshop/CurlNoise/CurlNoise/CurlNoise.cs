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
    private void RunScript(List<Point3d> P, double S, double t, bool D3, ref object V, ref object C)
    {

        // <Custom code>

        // test

        if (P == null || P.Count == 0) return;

        GH_Vector[] v = new GH_Vector[P.Count];
        GH_Colour[] c = new GH_Colour[P.Count];

        Parallel.For(0, P.Count, i => {
            CurlNoise(P[i], S, t, D3, out v[i], out c[i]);
        });

        
        V = v;
        C = c;

        // </Custom code>
    }

    // <Custom additional code> 

    public void CurlNoise(Point3d P, double S, double t, bool flag, out GH_Vector V, out GH_Colour C)
    {

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
        V = new GH_Vector(val2);
        val2.Unitize();
        int red = (int)Math.Floor(Remap((float)val2.X, -1f, 1f, 0f, 255f));
        int green = (int)Math.Floor(Remap((float)val2.Y, -1f, 1f, 0f, 255f));
        int blue = (int)Math.Floor(Remap((float)val2.Z, -1f, 1f, 0f, 255f));
        C = new GH_Colour(Color.FromArgb(red, green, blue));
   
    }

    public float Remap(float val, float from1, float to1, float from2, float to2)
    {
        return (val - from1) / (to1 - from1) * (to2 - from2) + from2;
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
        Point3d P = default(Point3d);
        if (inputs[0] != null)
        {
            P = (Point3d)(inputs[0]);
        }

        double S = default(double);
        if (inputs[1] != null)
        {
            S = (double)(inputs[1]);
        }

        double t = default(double);
        if (inputs[2] != null)
        {
            t = (double)(inputs[2]);
        }

        bool D3 = default(bool);
        if (inputs[3] != null)
        {
            D3 = (bool)(inputs[3]);
        }



        //3. Declare output parameters
        object V = null;
        object C = null;


        //4. Invoke RunScript
        RunScript(P, S, t, D3, ref V, ref C);

        try
        {
            //5. Assign output parameters to component...
            if (V != null)
            {
                if (GH_Format.TreatAsCollection(V))
                {
                    IEnumerable __enum_V = (IEnumerable)(V);
                    DA.SetDataList(1, __enum_V);
                }
                else
                {
                    if (V is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(V));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(1, V);
                    }
                }
            }
            else
            {
                DA.SetData(1, null);
            }
            if (C != null)
            {
                if (GH_Format.TreatAsCollection(C))
                {
                    IEnumerable __enum_C = (IEnumerable)(C);
                    DA.SetDataList(2, __enum_C);
                }
                else
                {
                    if (C is Grasshopper.Kernel.Data.IGH_DataTree)
                    {
                        //merge tree
                        DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(C));
                    }
                    else
                    {
                        //assign direct
                        DA.SetData(2, C);
                    }
                }
            }
            else
            {
                DA.SetData(2, null);
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