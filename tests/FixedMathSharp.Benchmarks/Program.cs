using BenchmarkDotNet.Running;
using System;
using System.Linq;

namespace FixedMathSharp.Benchmarks;

internal static class Program
{
    private static readonly BenchmarkCatalog _catalog = BenchmarkCatalog.Create(typeof(Program).Assembly);

    private static int Main(string[] args)
    {
        if (args.Length == 0)
        {
            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
            return 0;
        }

        string command = args[0];

        if ((string.Equals(command, "list", StringComparison.OrdinalIgnoreCase) ||
             string.Equals(command, "ls", StringComparison.OrdinalIgnoreCase)) &&
            args.Length == 1)
        {
            _catalog.WriteAvailableSelections(Console.Out);
            return 0;
        }

        if (string.Equals(command, "help", StringComparison.OrdinalIgnoreCase))
        {
            WriteUsage();
            return 0;
        }

        if (string.Equals(command, "all", StringComparison.OrdinalIgnoreCase))
        {
            if (args.Length > 1 && !args[1].StartsWith("-", StringComparison.Ordinal))
            {
                Console.Error.WriteLine("The 'all' selection cannot be combined with other benchmark aliases.");
                Console.Error.WriteLine();
                WriteUsage();
                return 1;
            }

            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(EnsureAllBenchmarksSelected(args.Skip(1).ToArray()));
            return 0;
        }

        int aliasCount = 0;
        while (aliasCount < args.Length && !args[aliasCount].StartsWith("-", StringComparison.Ordinal))
            aliasCount++;

        if (aliasCount == 0)
        {
            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
            return 0;
        }

        Type[] selectedTypes = _catalog.Resolve(args.Take(aliasCount).ToArray(), out string unknownAlias);
        if (unknownAlias != null)
        {
            Console.Error.WriteLine($"Unknown benchmark selection '{unknownAlias}'.");
            Console.Error.WriteLine();
            WriteUsage();
            return 1;
        }

        BenchmarkSwitcher.FromTypes(selectedTypes).Run(EnsureAllBenchmarksSelected(args.Skip(aliasCount).ToArray()));
        return 0;
    }

    private static string[] EnsureAllBenchmarksSelected(string[] benchmarkArgs)
    {
        if (benchmarkArgs.Any(IsBenchmarkSelectionArgument))
            return benchmarkArgs;

        return new[] { "--filter", "*" }.Concat(benchmarkArgs).ToArray();
    }

    private static bool IsBenchmarkSelectionArgument(string argument)
    {
        return string.Equals(argument, "--filter", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "-f", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--list", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--help", StringComparison.OrdinalIgnoreCase)
            || string.Equals(argument, "--version", StringComparison.OrdinalIgnoreCase);
    }

    private static void WriteUsage()
    {
        Console.WriteLine("Usage:");
        Console.WriteLine("  dotnet build tests/FixedMathSharp.Benchmarks/FixedMathSharp.Benchmarks.csproj -c Release -f net8.0");
        Console.WriteLine("  dotnet tests/FixedMathSharp.Benchmarks/bin/Release/net8.0/FixedMathSharp.Benchmarks.dll all --list flat");
        Console.WriteLine();
        Console.WriteLine("Leading arguments that do not start with '-' are treated as benchmark selections.");
        Console.WriteLine("Remaining arguments are forwarded to BenchmarkDotNet.");
        Console.WriteLine();
        _catalog.WriteAvailableSelections(Console.Out);
    }
}
