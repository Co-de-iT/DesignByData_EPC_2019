using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



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
  private void RunScript(List<Point3d> P, List<Vector3d> V, List<Vector3d> vF, double pR, double cI, double aI, double sI, object fI, double mF, ref object Planes)
  {
    // align planes with field - C#
  }

  // <Custom additional code> 
  
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
    this. doc = this.RhinoDocument;

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