using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drone : MonoBehaviour
{
    // State vectors
    private Vector3 n; // Global simulation frame position [x, y, psi]
    private Vector3 nDot; // Global simulation frame velocity [xDot, yDot, psiDot]
    private Vector3 v; // Local body frame velocity [u, v, r]
    private Vector3 vDot; // Local body frame acceleration [uDot, vDot, rDot]

    // Forces
    private Vector3 tau; // Local body frame internal forces [Fx, Fy, T]
    private Vector3 tauW; // Local body frame external forces [Fwx, Fwy, 0]

    // Parameters
    public float forwardEngineForce = 24.5f;
    public float backwardEngineForce = 24.5f;
    public float engineDistance = 0.98f;
    public float timeStep = 0.01f;

    // Inertia matrix
    private Matrix3x3 inertiaMatrix;
    private Matrix3x3 inertiaMatrixInv;

    void Start()
    {
        // Initial conditions
        n = new Vector3(0f, 0f, 0f);
        v = new Vector3(0f, 0f, 0f);
        nDot = new Vector3(0f, 0f, 0f);
        vDot = new Vector3(0f, 0f, 0f);

        // Internal forces initialization
        tau = new Vector3(0f, 0f, 0f);

        // External forces initialization
        tauW = new Vector3(0f, 0f, 0f);

        // Inertia matrix initialization
        inertiaMatrix = Matrix3x3.InertiaMatrix();
        inertiaMatrixInv = inertiaMatrix.Inverse();
    }

    void FixedUpdate()
    {
        // Calculate acceleration
        vDot = CalculateAcceleration();

        // Update velocity
        v += vDot * timeStep;

        // Transform velocity to global frame using a 3x3 transformation matrix
        Matrix3x3 T = CreateTransformationMatrix(n[2]);
        Vector3 globalVelocity = T * v;

        // Update global velocity components
        nDot[0] = globalVelocity[0];
        nDot[1] = globalVelocity[1];
        nDot[2] = v[2]; // Angular velocity remains unchanged

        // Update position
        n += nDot * timeStep;

        // Transform position to Unity frame
        UpdateUnityFrame(n[0], n[1], n[2]);
    }

    void UpdateUnityFrame(float x, float y, float psi)
    {
        // Transform position from global frame to Unity frame
        // x -> z, y -> x
        transform.position = new Vector3(y, 0f, x);

        // Transform rotation from global frame to Unity frame
        transform.rotation = Quaternion.Euler(0f, psi * Mathf.Rad2Deg, 0f);
    }

    Vector3 CalculateAcceleration()
    {
        // Initialize input forces
        tau = Vector3.zero;

        // Initialize engine forces
        float Fr = 0f;
        float Fl = 0f;

        // Handle inputs
        if (Input.GetKey(KeyCode.E))
        {
            Fr = forwardEngineForce;
        }

        if (Input.GetKey(KeyCode.D))
        {
            Fr = -backwardEngineForce;
        }

        if (Input.GetKey(KeyCode.Q))
        {
            Fl = forwardEngineForce;
        }

        if (Input.GetKey(KeyCode.A))
        {
            Fl = -backwardEngineForce;
        }

        // Calculate total thrust
        tau.x = Fr + Fl;

        // Calculate total torque
        tau.z = (Fr - Fl) * (engineDistance / 2);

        // Calculate Coriolis matrix
        Matrix3x3 C = Matrix3x3.CoriolisMatrix(v, inertiaMatrix);

        // Calculate damping matrix
        Matrix3x3 D = Matrix3x3.DampingMatrix(v);

        // Compute the right-hand side: tau + tauW - C*v - D*v
        Vector3 rhs = tau + tauW - (C * v) * 0 - (D * v);

        // Calculate acceleration
        vDot = inertiaMatrixInv * rhs;

        return vDot;
    }

    // Create a 3x3 transformation matrix for 2D rotation
    Matrix3x3 CreateTransformationMatrix(float psi)
    {
        Matrix3x3 T = new Matrix3x3();

        // Rotation matrix for 2D plane
        T[0, 0] = Mathf.Cos(psi);
        T[0, 1] = -Mathf.Sin(psi);
        T[1, 0] = Mathf.Sin(psi);
        T[1, 1] = Mathf.Cos(psi);

        // No translation needed, so 3rd row and column are identity
        T[2, 2] = 1;

        return T;
    }
}

// Custom 3x3 matrix class with operator overloading
public class Matrix3x3
{
    private float[,] matrix;

    public Matrix3x3()
    {
        matrix = new float[3, 3];
    }

    public float this[int row, int col]
    {
        get { return matrix[row, col]; }
        set { matrix[row, col] = value; }
    }
    
    // Overload the * operator for Matrix3x3 and Vector3 multiplication
    public static Vector3 operator *(Matrix3x3 lhs, Vector3 rhs)
    {
        return new Vector3(
            lhs[0, 0] * rhs.x + lhs[0, 1] * rhs.y + lhs[0, 2] * rhs.z,
            lhs[1, 0] * rhs.x + lhs[1, 1] * rhs.y + lhs[1, 2] * rhs.z,
            lhs[2, 0] * rhs.x + lhs[2, 1] * rhs.y + lhs[2, 2] * rhs.z
        );
    }

    // Overload the - (unary negation) operator for Matrix3x3
    public static Matrix3x3 operator -(Matrix3x3 matrix)
    {
        Matrix3x3 result = new Matrix3x3();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result[i, j] = -matrix[i, j];
            }
        }
        return result;
    }

    // Inverse of a 3x3 matrix
    public Matrix3x3 Inverse()
    {
        Matrix3x3 result = new Matrix3x3();
        
        // Calculate the determinant of the matrix
        float det = matrix[0, 0] * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1])
                - matrix[0, 1] * (matrix[1, 0] * matrix[2, 2] - matrix[1, 2] * matrix[2, 0])
                + matrix[0, 2] * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);

        if (Mathf.Abs(det) < Mathf.Epsilon) // Check if the determinant is 0
        {
            Debug.LogError("Matrix is not invertible");
            return null;
        }

        // Calculate the cofactor matrix
        result[0, 0] = (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]) / det;
        result[0, 1] = (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]) / det;
        result[0, 2] = (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]) / det;

        result[1, 0] = (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]) / det;
        result[1, 1] = (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]) / det;
        result[1, 2] = (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]) / det;

        result[2, 0] = (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]) / det;
        result[2, 1] = (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]) / det;
        result[2, 2] = (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]) / det;

        return result;
    }

    // Inertia matrix
    public static Matrix3x3 InertiaMatrix()
    {
        Matrix3x3 inertiaMatrix = new Matrix3x3();
        inertiaMatrix[0, 0] = 25.8f;
        inertiaMatrix[0, 1] = 0f;
        inertiaMatrix[0, 2] = 0f;
        inertiaMatrix[1, 0] = 0f;
        inertiaMatrix[1, 1] = 33.8f;
        inertiaMatrix[1, 2] = 1.0948f;
        inertiaMatrix[2, 0] = 0f;
        inertiaMatrix[2, 1] = 1.0948f;
        inertiaMatrix[2, 2] = 2.76f;

        return inertiaMatrix;
    }

    // Damping matrix
    public static Matrix3x3 DampingMatrix(Vector3 v)
    {
        Matrix3x3 dampingMatrix = new Matrix3x3();
        
        float u = v[0];
        float v_y = v[1];
        float r = v[2];

        dampingMatrix[0, 0] = 0.72f + 1.33f * Mathf.Abs(u);
        dampingMatrix[0, 1] = 0f;
        dampingMatrix[0, 2] = 0f;

        dampingMatrix[1, 0] = 0f;
        dampingMatrix[1, 1] = 0.9f + 36.5f * Mathf.Abs(v_y) + 0.8f * Mathf.Abs(r);
        dampingMatrix[1, 2] = 7.2f + 0.85f * Mathf.Abs(v_y) + 3.5f * Mathf.Abs(r);

        dampingMatrix[2, 0] = 0f;
        dampingMatrix[2, 1] = -0.03f - 4f * Mathf.Abs(v_y) - 0.13f * Mathf.Abs(r);
        dampingMatrix[2, 2] = 1.9f - 0.08f * Mathf.Abs(v_y) + 0.75f * Mathf.Abs(r);

        return dampingMatrix;
    }

    // Coriolis matrix
    public static Matrix3x3 CoriolisMatrix(Vector3 v, Matrix3x3 M)
    {
        Matrix3x3 coriolisMatrix = new Matrix3x3();

        float u = v[0];
        float v_y = v[1];
        float r = v[2];

        coriolisMatrix[0, 0] = 0f;
        coriolisMatrix[0, 1] = 0f;
        coriolisMatrix[0, 2] = -M[1, 1] * v_y - M[1, 2] * r;

        coriolisMatrix[1, 0] = 0f;
        coriolisMatrix[1, 1] = 0f;
        coriolisMatrix[1, 2] = M[0, 0] * u;

        coriolisMatrix[2, 0] = -(-M[1, 1] * v_y - M[1, 2] * r);
        coriolisMatrix[2, 1] = -M[0, 0] * u;
        coriolisMatrix[2, 2] = 0f;

        return coriolisMatrix;
    }
}
