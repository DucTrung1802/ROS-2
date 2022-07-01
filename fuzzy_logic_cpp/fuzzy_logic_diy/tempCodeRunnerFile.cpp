[] = {1, 2, 3, 4, 6, 8, 9, 4, 5};

    int *min = std::min_element(playerSums, playerSums + 3);

    std::cout << playerNames[min - playerSums] << " had the lowest values and got the sum " << *min << std::endl;