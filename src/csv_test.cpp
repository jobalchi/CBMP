#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {
    // Variables to store each vector (of type double)
    std::vector<double> cx, cy, cyaw, ck;

    // Open the file
    std::ifstream file("data.csv"); // Change the file name as needed
    std::string line;

    // Skip the first three lines (metadata)
    for (int i = 0; i < 3; i++) {
        std::getline(file, line);
    }

    // Read the data
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        // Split by semicolon
        while (std::getline(ss, value, ';')) {
            row.push_back(std::stod(value)); // Convert string to double
        }

        // Assign data to corresponding columns
        cx.push_back(row[1]);   // x_m
        cy.push_back(row[2]);   // y_m
        cyaw.push_back(row[3]); // psi_rad (yaw angle)
        ck.push_back(row[4]);   // kappa_radpm (curvature)
    }

    // Output the result (for verification)
    for (size_t i = 0; i < cx.size(); ++i) {
        std::cout << "cx: " << cx[i] << ", cy: " << cy[i] << ", cyaw: " << cyaw[i] << ", ck: " << ck[i] << std::endl;
    }

    file.close();
    return 0;
}
