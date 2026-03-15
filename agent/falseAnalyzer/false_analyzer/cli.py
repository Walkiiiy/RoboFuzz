import argparse
import os

from .collector import collect_run
from .llm_reasoner import enrich_with_llm
from .reporter import write_case, write_summary
from .rule_triager import triage_case


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True, help="Path to logs/<run_id>")
    parser.add_argument("--output", required=True, help="Output directory root")
    parser.add_argument(
        "--use-llm",
        action="store_true",
        help="Call DeepSeek after rule-based triage",
    )
    parser.add_argument("--api-key", help="DeepSeek API key", default=None)
    parser.add_argument("--base-url", help="DeepSeek base URL", default=None)
    parser.add_argument("--model", help="DeepSeek model name", default=None)
    args = parser.parse_args()

    bundles = collect_run(args.run)
    run_id = os.path.basename(args.run.rstrip("/"))
    run_output = os.path.join(args.output, run_id)

    results = []
    for bundle in bundles:
        triage = triage_case(bundle)
        try:
            triage = enrich_with_llm(
                bundle,
                triage,
                use_llm=args.use_llm,
                api_key=args.api_key,
                base_url=args.base_url,
                model=args.model,
            )
        except Exception as exc:
            triage.notes.append(f"llm_error: {exc}")
        write_case(run_output, triage)
        results.append(triage)

    write_summary(run_output, results)


if __name__ == "__main__":
    main()
