"""
ga.py - Permutation-Based Genetic Algorithm for UAV Coverage Path Optimization
"""
from typing import List, Tuple, Optional
import random
import numpy as np
from config import GAConfig
from fitness import FitnessEvaluator, EvaluationMetrics


class GeneticAlgorithm:
    """
    Permutation-based Genetic Algorithm implementing:
    - Random/seeded initialization
    - Tournament selection
    - Order Crossover (OX)
    - Inversion / Swap mutation
    - Elitism
    - Convergence logging
    """

    def __init__(self,
                 valid_cell_ids: List[int],
                 evaluator: FitnessEvaluator,
                 ga_cfg: Optional[GAConfig] = None,
                 seed: int = 42):
        self.valid_cell_ids = list(valid_cell_ids)
        self.num_genes = len(self.valid_cell_ids)
        self.evaluator = evaluator
        self.cfg = ga_cfg or GAConfig()
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)

        self.convergence_history: List[float] = []

    def initialize_population(self, seed_individuals: Optional[List[List[int]]] = None) -> List[List[int]]:
        """
        Create initial population. If seed_individuals are provided (e.g. from ACO or baseline),
        they are included first, and the rest is filled with random valid permutations.
        """
        pop: List[List[int]] = []
        if seed_individuals:
            for ind in seed_individuals:
                if len(ind) == self.num_genes and set(ind) == set(self.valid_cell_ids):
                    pop.append(list(ind))
                if len(pop) >= self.cfg.population_size:
                    break

        while len(pop) < self.cfg.population_size:
            ind = list(self.valid_cell_ids)
            self.rng.shuffle(ind)
            repaired_ind = self.evaluator.grid.repair_obstacle_collisions(ind)
            pop.append(repaired_ind)

        return pop

    def tournament_selection(self, population: List[List[int]], costs: List[float]) -> List[int]:
        """Select individual with lowest cost among k random contestants."""
        contestants_idx = self.rng.sample(range(len(population)), self.cfg.tournament_size)
        best_idx = min(contestants_idx, key=lambda idx: costs[idx])
        return list(population[best_idx])

    def order_crossover(self, p1: List[int], p2: List[int]) -> Tuple[List[int], List[int]]:
        """
        Order Crossover (OX-1) preserving permutation constraints.
        Ensures offspring have no duplicates and no missing cells.
        """
        n = self.num_genes
        if n < 3:
            return list(p1), list(p2)

        idx1, idx2 = sorted(self.rng.sample(range(n), 2))

        def ox_child(parent1: List[int], parent2: List[int]) -> List[int]:
            child = [-1] * n
            # Copy subsegment from parent1
            child[idx1:idx2] = parent1[idx1:idx2]
            copied_set = set(child[idx1:idx2])

            # Fill remaining from parent2 in order starting after idx2
            p2_ordered = parent2[idx2:] + parent2[:idx2]
            fill_values = [gene for gene in p2_ordered if gene not in copied_set]

            # Fill child after idx2, then wrap around
            fill_indices = list(range(idx2, n)) + list(range(0, idx1))
            for c_idx, val in zip(fill_indices, fill_values):
                child[c_idx] = val

            return child

        c1 = ox_child(p1, p2)
        c2 = ox_child(p2, p1)
        return c1, c2

    def mutate(self, ind: List[int]) -> List[int]:
        """
        Permutation mutation: with equal probability either segment inversion (2-opt)
        or pair swap.
        """
        mutated = list(ind)
        n = self.num_genes
        if n < 2:
            return mutated

        if self.rng.random() < 0.5:
            # Inversion mutation
            i, j = sorted(self.rng.sample(range(n), 2))
            mutated[i:j + 1] = reversed(mutated[i:j + 1])
        else:
            # Pair swap mutation
            i, j = self.rng.sample(range(n), 2)
            mutated[i], mutated[j] = mutated[j], mutated[i]

        return mutated

    def optimize(self, seed_individuals: Optional[List[List[int]]] = None) -> Tuple[List[int], EvaluationMetrics]:
        """
        Execute Genetic Algorithm optimization loop.
        Returns: (best_path, best_metrics)
        """
        population = self.initialize_population(seed_individuals)
        costs = [self.evaluator(ind) for ind in population]

        best_idx = int(np.argmin(costs))
        best_individual = list(population[best_idx])
        best_cost = costs[best_idx]

        self.convergence_history = [best_cost]

        for gen in range(self.cfg.generations):
            # Elitism: retain top elite_count individuals
            sorted_indices = np.argsort(costs)
            next_pop = [list(population[i]) for i in sorted_indices[:self.cfg.elite_count]]

            # Reproduction loop
            while len(next_pop) < self.cfg.population_size:
                p1 = self.tournament_selection(population, costs)
                p2 = self.tournament_selection(population, costs)

                # Crossover
                if self.rng.random() < self.cfg.crossover_prob:
                    c1, c2 = self.order_crossover(p1, p2)
                else:
                    c1, c2 = list(p1), list(p2)

                # Mutation
                if self.rng.random() < self.cfg.mutation_prob:
                    c1 = self.mutate(c1)
                if self.rng.random() < self.cfg.mutation_prob:
                    c2 = self.mutate(c2)

                # Stage 6: Repair (untangle any obstacle crossing segments)
                c1 = self.evaluator.grid.repair_obstacle_collisions(c1)
                c2 = self.evaluator.grid.repair_obstacle_collisions(c2)

                next_pop.append(c1)
                if len(next_pop) < self.cfg.population_size:
                    next_pop.append(c2)

            population = next_pop
            costs = [self.evaluator(ind) for ind in population]

            current_best_idx = int(np.argmin(costs))
            if costs[current_best_idx] < best_cost:
                best_cost = costs[current_best_idx]
                best_individual = list(population[current_best_idx])

            self.convergence_history.append(best_cost)

        best_metrics = self.evaluator.evaluate_path(best_individual)
        return best_individual, best_metrics
