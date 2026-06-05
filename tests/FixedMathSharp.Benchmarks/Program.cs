using BenchmarkDotNet.Running;
using System;

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

            BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(EnsureAllBenchmarksSelected(CopyRange(args, 1, args.Length - 1)));
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

        Type[] selectedTypes = _catalog.Resolve(CopyRange(args, 0, aliasCount), out string unknownAlias);
        if (unknownAlias != null)
        {
            Console.Error.WriteLine($"Unknown benchmark selection '{unknownAlias}'.");
            Console.Error.WriteLine();
            WriteUsage();
            return 1;
        }

        BenchmarkSwitcher.FromTypes(selectedTypes).Run(EnsureAllBenchmarksSelected(CopyRange(args, aliasCount, args.Length - aliasCount)));
        return 0;
    }

    private static string[] EnsureAllBenchmarksSelected(string[] benchmarkArgs)
    {
        for (int i = 0; i < benchmarkArgs.Length; i++)
        {
            if (IsBenchmarkSelectionArgument(benchmarkArgs[i]))
                return benchmarkArgs;
        }

        var args = new string[benchmarkArgs.Length + 2];
        args[0] = "--filter";
        args[1] = "*";
        Array.Copy(benchmarkArgs, 0, args, 2, benchmarkArgs.Length);
        return args;
    }

    private static string[] CopyRange(string[] values, int startIndex, int length)
    {
        if (length == 0)
            return Array.Empty<string>();

        var range = new string[length];
        Array.Copy(values, startIndex, range, 0, length);
        return range;
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
