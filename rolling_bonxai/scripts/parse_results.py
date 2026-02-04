#!/usr/bin/env python3
"""
parse_benchmark_results.py

Parses nanobench output from Rolling Bonxai chunk storage benchmarks.
Handles warnings, unstable results, and special formatting.

Usage:
    python3 parse_benchmark_results.py <input_file> [options]
    
Example:
    python3 parse_benchmark_results.py results.txt --output chart.png --csv results.csv
"""

import re
import sys
import argparse
from typing import List, Dict, Tuple
from dataclasses import dataclass
from pathlib import Path


@dataclass
class BenchmarkResult:
    """Single benchmark result"""
    storage_type: str      # Vector, HashMap, Deque
    operation: str         # insert, find_hit_first, erase_middle, etc.
    radius: int           # 1-5
    num_chunks: int       # 27, 125, 343, 729, 1331
    ns_per_op: float      # Time in nanoseconds
    ops_per_sec: float    # Throughput
    error_pct: float      # Error percentage
    is_unstable: bool     # Whether result had unstable warning


def parse_benchmark_line(line: str) -> Tuple[str, Dict[str, float]]:
    """
    Parse a single benchmark result line.
    
    Example input:
    |           76,464.31 |           13,078.00 |    1.9% |      0.94 | `Vector insert (r=1, n=27)`
    
    Returns:
        (benchmark_name, metrics_dict)
    """
    # Remove wavy dash markers for unstable results
    line = line.replace(':wavy_dash:', '').strip()
    
    # Split by pipe and clean up
    parts = [p.strip() for p in line.split('|')]
    
    # Should have format: | ns/op | op/s | err% | total | name |
    if len(parts) < 6:
        return None, {}
    
    try:
        # Extract metrics (handle comma separators)
        ns_per_op = float(parts[1].replace(',', ''))
        ops_per_sec = float(parts[2].replace(',', ''))
        error_pct = float(parts[3].replace('%', ''))
        
        # Extract benchmark name (remove backticks and extra markers)
        name = parts[5]
        name = name.replace('`', '').strip()
        
        # Check if unstable
        is_unstable = 'Unstable' in line or 'minEpochIterations' in line
        
        # Clean name further (remove unstable warnings)
        if '(' in name and 'Unstable' in name:
            name = name.split('(Unstable')[0].strip()
        
        metrics = {
            'ns_per_op': ns_per_op,
            'ops_per_sec': ops_per_sec,
            'error_pct': error_pct,
            'is_unstable': is_unstable
        }
        
        return name, metrics
        
    except (ValueError, IndexError) as e:
        return None, {}


def extract_benchmark_info(name: str) -> Dict[str, any]:
    """
    Extract storage type, operation, radius, and chunk count from benchmark name.
    
    Example: "Vector insert (r=1, n=27)"
    Returns: {
        'storage_type': 'Vector',
        'operation': 'insert',
        'radius': 1,
        'num_chunks': 27
    }
    """
    # Pattern: "StorageType operation (r=X, n=Y)"
    pattern = r'(\w+)\s+(.+?)\s+\(r=(\d+),\s*n=(\d+)\)'
    match = re.match(pattern, name)
    
    if not match:
        return None
    
    storage_type = match.group(1)
    operation = match.group(2).strip()
    radius = int(match.group(3))
    num_chunks = int(match.group(4))
    
    return {
        'storage_type': storage_type,
        'operation': operation,
        'radius': radius,
        'num_chunks': num_chunks
    }


def parse_benchmark_file(filepath: str) -> List[BenchmarkResult]:
    """
    Parse entire benchmark output file.
    
    Returns list of BenchmarkResult objects.
    """
    results = []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Skip empty lines, headers, and separator lines
            if not line or line.startswith('=') or line.startswith('---'):
                continue
            
            # Skip warning lines
            if 'Warning' in line or 'CPU frequency' in line or 'Recommendations' in line:
                continue
            
            # Skip header row
            if 'ns/op' in line and 'op/s' in line and 'err%' in line:
                continue
            
            # Parse data line (starts with |)
            if line.startswith('|'):
                name, metrics = parse_benchmark_line(line)
                
                if name and metrics:
                    info = extract_benchmark_info(name)
                    
                    if info:
                        result = BenchmarkResult(
                            storage_type=info['storage_type'],
                            operation=info['operation'],
                            radius=info['radius'],
                            num_chunks=info['num_chunks'],
                            ns_per_op=metrics['ns_per_op'],
                            ops_per_sec=metrics['ops_per_sec'],
                            error_pct=metrics['error_pct'],
                            is_unstable=metrics['is_unstable']
                        )
                        results.append(result)
    
    return results


def export_to_csv(results: List[BenchmarkResult], filepath: str):
    """Export results to CSV file."""
    import csv
    
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Header
        writer.writerow([
            'Storage Type',
            'Operation',
            'Radius',
            'Num Chunks',
            'Time (ns/op)',
            'Throughput (op/s)',
            'Error (%)',
            'Unstable'
        ])
        
        # Data rows
        for r in results:
            writer.writerow([
                r.storage_type,
                r.operation,
                r.radius,
                r.num_chunks,
                f'{r.ns_per_op:.2f}',
                f'{r.ops_per_sec:.2f}',
                f'{r.error_pct:.2f}',
                'Yes' if r.is_unstable else 'No'
            ])
    
    print(f"✓ CSV exported to: {filepath}")


def generate_charts(results: List[BenchmarkResult], filepath: str):
    """Generate comparison charts."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("✗ matplotlib not installed. Skipping chart generation.")
        print("  Install with: pip3 install matplotlib")
        return
    
    # Group results by operation type
    operations = {
        'insert': [],
        'find_hit_first': [],
        'find_miss': [],
        'erase_first': [],
        'iterate': [],
        'mixed_workload': []
    }
    
    for r in results:
        for op_key in operations.keys():
            if op_key in r.operation:
                operations[op_key].append(r)
                break
    
    # Create 2x3 subplot layout
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Rolling Bonxai Chunk Storage Benchmark Comparison', fontsize=16, fontweight='bold')
    
    storage_types = ['Vector', 'HashMap', 'Deque']
    colors = {'Vector': '#3498db', 'HashMap': '#e74c3c', 'Deque': '#2ecc71'}
    
    plot_configs = [
        ('insert', axes[0, 0], 'Insert Performance'),
        ('find_hit_first', axes[0, 1], 'Find (Hit) Performance'),
        ('find_miss', axes[0, 2], 'Find (Miss) Performance'),
        ('erase_first', axes[1, 0], 'Erase Performance'),
        ('iterate', axes[1, 1], 'Iteration Performance'),
        ('mixed_workload', axes[1, 2], 'Mixed Workload Performance')
    ]
    
    for op_key, ax, title in plot_configs:
        op_results = operations[op_key]
        
        if not op_results:
            ax.text(0.5, 0.5, 'No data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title(title)
            continue
        
        # Group by storage type
        for storage in storage_types:
            storage_data = [r for r in op_results if r.storage_type == storage]
            storage_data.sort(key=lambda x: x.radius)
            
            if storage_data:
                radii = [r.radius for r in storage_data]
                times_us = [r.ns_per_op / 1000.0 for r in storage_data]  # Convert to microseconds
                
                ax.plot(radii, times_us, marker='o', label=storage, 
                       color=colors[storage], linewidth=2, markersize=8)
        
        ax.set_xlabel('Radius (chunks)', fontsize=10)
        ax.set_ylabel('Time (μs/op)', fontsize=10)
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_yscale('log')  # Log scale for better visibility
    
    plt.tight_layout()
    plt.savefig(filepath, dpi=300, bbox_inches='tight')
    print(f"✓ Chart saved to: {filepath}")


def print_summary(results: List[BenchmarkResult]):
    """Print summary statistics."""
    print("\n" + "="*80)
    print("  BENCHMARK SUMMARY")
    print("="*80)
    
    # Group by radius
    radii = sorted(set(r.radius for r in results))
    
    print(f"\nTotal benchmarks: {len(results)}")
    print(f"Radii tested: {radii}")
    print(f"Storage types: Vector, HashMap, Deque")
    
    # Find fastest for each operation at radius=1
    print("\n" + "-"*80)
    print("Fastest at Radius 1 (27 chunks):")
    print("-"*80)
    
    r1_results = [r for r in results if r.radius == 1]
    operations_r1 = {}
    
    for r in r1_results:
        if r.operation not in operations_r1:
            operations_r1[r.operation] = []
        operations_r1[r.operation].append(r)
    
    for op, op_results in sorted(operations_r1.items()):
        fastest = min(op_results, key=lambda x: x.ns_per_op)
        print(f"  {op:25s} : {fastest.storage_type:10s} ({fastest.ns_per_op:12,.2f} ns)")
    
    # HashMap vs Vector speedup at different radii
    print("\n" + "-"*80)
    print("HashMap vs Vector Speedup (insert operation):")
    print("-"*80)
    
    for radius in radii:
        vector_insert = next((r for r in results 
                            if r.radius == radius and r.storage_type == 'Vector' 
                            and r.operation == 'insert'), None)
        hashmap_insert = next((r for r in results 
                             if r.radius == radius and r.storage_type == 'HashMap' 
                            and r.operation == 'insert'), None)
        
        if vector_insert and hashmap_insert:
            speedup = vector_insert.ns_per_op / hashmap_insert.ns_per_op
            print(f"  Radius {radius} ({vector_insert.num_chunks:4d} chunks): "
                  f"HashMap is {speedup:5.2f}x faster")
    
    # Unstable results warning
    unstable = [r for r in results if r.is_unstable]
    if unstable:
        print(f"\n⚠  Warning: {len(unstable)} unstable results detected")
        print("   Consider running with CPU governor set to 'performance'")


def main():
    parser = argparse.ArgumentParser(
        description='Parse Rolling Bonxai benchmark results',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 parse_benchmark_results.py results.txt
  python3 parse_benchmark_results.py results.txt --csv data.csv
  python3 parse_benchmark_results.py results.txt --output chart.png --csv data.csv
        """
    )
    
    parser.add_argument('input', help='Input benchmark results file')
    parser.add_argument('--csv', help='Output CSV file')
    parser.add_argument('--output', help='Output chart PNG file')
    parser.add_argument('--no-summary', action='store_true', 
                       help='Skip printing summary')
    
    args = parser.parse_args()
    
    # Check input file exists
    if not Path(args.input).exists():
        print(f"✗ Error: Input file not found: {args.input}")
        sys.exit(1)
    
    print(f"Parsing benchmark results from: {args.input}")
    
    # Parse results
    try:
        results = parse_benchmark_file(args.input)
    except Exception as e:
        print(f"✗ Error parsing file: {e}")
        sys.exit(1)
    
    if not results:
        print("✗ No benchmark results found in file")
        sys.exit(1)
    
    print(f"✓ Parsed {len(results)} benchmark results")
    
    # Export CSV
    if args.csv:
        try:
            export_to_csv(results, args.csv)
        except Exception as e:
            print(f"✗ Error exporting CSV: {e}")
    
    # Generate charts
    if args.output:
        try:
            generate_charts(results, args.output)
        except Exception as e:
            print(f"✗ Error generating chart: {e}")
    
    # Print summary
    if not args.no_summary:
        print_summary(results)
    
    print("\n✓ Done!")


if __name__ == '__main__':
    main()