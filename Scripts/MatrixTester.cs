using UnityEngine;

public class MatrixTester : MonoBehaviour
{
    void Start()
    {
        // Create a new Matrix3x3 instance
        Matrix3x3 matrix = new Matrix3x3();

        // Set the matrix elements
        matrix[0, 0] = 1f;
        matrix[0, 1] = 2f;
        matrix[0, 2] = 3f;

        matrix[1, 0] = 3f;
        matrix[1, 1] = 2f;
        matrix[1, 2] = 1f;

        matrix[2, 0] = 2f;
        matrix[2, 1] = 1f;
        matrix[2, 2] = 3f;

        // Compute the inverse
        Matrix3x3 inverse = matrix.Inverse();

        // Check if the inverse is valid
        if (inverse != null)
        {
            Debug.Log("Inverse matrix:");
            // Print the inverse matrix
            for (int i = 0; i < 3; i++)
            {
                string row = "";
                for (int j = 0; j < 3; j++)
                {
                    row += inverse[i, j].ToString("F4") + " ";
                }
                Debug.Log(row);
            }
        }
    }
}
